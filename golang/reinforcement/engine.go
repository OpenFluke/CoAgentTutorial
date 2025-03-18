package main

import (
	"bufio"
	"encoding/json"
	"errors"
	"fmt"
	"log"
	"math"
	"math/rand"
	"net"
	"os"
	"strings"
	"sync"
	"time"

	// Paragon for feedforward networks
	paragon "github.com/OpenFluke/PARAGON"
)

// -----------------------------
// Adjustable Settings
// -----------------------------

// Network / training hyperparams
const (
	NumAgents      = 20       // how many agents (threads) to run
	Episodes       = 30       // how many episodes per agent
	MaxSteps       = 300      // how many steps per episode if not done
	PolicyMode     = "hybrid" // "ddpg", "heuristic", or "hybrid"
	BatchSize      = 64       // replay mini-batch
	ReplayCapacity = 100000
	ActorHidden    = 128
	CriticHidden   = 128
	ActionDim      = 3
	StateDim       = 6

	// If an episode runs longer than this, we stop.
	MaxEpisodeDuration = 20 * time.Second

	// We add +2.0 to each action to give a stronger push.
	ActionOffset = 2.0
)

// Environment constants
const (
	SERVER_IP       = "192.168.0.116"
	SERVER_PORT     = 14000
	SHARED_PASSWORD = "my_secure_password"
	END_OF_MSG      = "<???DONE???---"

	// Timeout for reading from TCP socket
	SOCKET_TIMEOUT = 1 * time.Second
	// Wait times after commands
	ResetWait = 500 * time.Millisecond
	StepWait  = 100 * time.Millisecond
)

// Game environment reward/scaling
const (
	Tolerance         = 5.0
	FarThreshold      = 120.0
	BonusReward       = 50.0
	PenaltyReward     = -50.0
	RewardScale       = 20.0
	StepPenalty       = 0.5
	InactionThreshold = 0.1
	InactionPenalty   = 2.0
)

// DDPG hyperparams
const (
	ActorLR       = 1e-4
	CriticLR      = 1e-3
	Gamma         = 0.99
	Tau           = 0.005
	HybridWeight  = 0.7
	NoiseTheta    = 0.15
	NoiseSigma    = 0.2
	HeuristicGain = 0.05
)

// -----------------------------
// Socket send/receive
// -----------------------------
func sendMessage(conn net.Conn, message string) error {
	fullMessage := message + END_OF_MSG
	_, err := conn.Write([]byte(fullMessage))
	return err
}

func receiveMessage(conn net.Conn) (string, error) {
	reader := bufio.NewReader(conn)
	var sb strings.Builder

	for {
		conn.SetReadDeadline(time.Now().Add(SOCKET_TIMEOUT))
		chunk, err := reader.ReadString('\n')
		if err != nil && !errors.Is(err, os.ErrDeadlineExceeded) {
			if !strings.Contains(err.Error(), "timeout") {
				return sb.String(), err
			}
		}
		sb.WriteString(chunk)
		if strings.Contains(sb.String(), END_OF_MSG) {
			break
		}
		if len(chunk) == 0 {
			break
		}
	}
	result := strings.ReplaceAll(sb.String(), END_OF_MSG, "")
	return strings.TrimSpace(result), nil
}

// -----------------------------
// GameCubeEnv
// -----------------------------
type GameCubeEnv struct {
	conn net.Conn

	target   [3]float64
	prevDist float64
	state    []float64
}

func NewGameCubeEnv() *GameCubeEnv {
	return &GameCubeEnv{
		target: [3]float64{69.58438552780794, 72.56208670917846, 30.29450219308874},
		state:  make([]float64, StateDim),
	}
}

func (env *GameCubeEnv) connect() error {
	address := fmt.Sprintf("%s:%d", SERVER_IP, SERVER_PORT)
	c, err := net.Dial("tcp", address)
	if err != nil {
		return err
	}
	env.conn = c
	if err := sendMessage(c, SHARED_PASSWORD); err != nil {
		return err
	}
	authResp, err := receiveMessage(c)
	if err != nil {
		return err
	}
	if authResp != "auth_success" {
		return fmt.Errorf("authentication failed in environment")
	}
	return nil
}

func (env *GameCubeEnv) disconnect() {
	if env.conn != nil {
		env.conn.Close()
		env.conn = nil
	}
}

func (env *GameCubeEnv) Reset() ([]float64, error) {
	env.disconnect()
	if err := env.connect(); err != nil {
		return nil, fmt.Errorf("env connect error: %w", err)
	}

	spawnCmd := map[string]interface{}{
		"type":     "spawn_cube",
		"position": []float64{0, 100, 40},
		"rotation": []float64{0, 0, 0},
	}
	spawnJSON, _ := json.Marshal(spawnCmd)
	if err := sendMessage(env.conn, string(spawnJSON)); err != nil {
		return nil, err
	}
	time.Sleep(ResetWait)

	getStateCmd := map[string]interface{}{"type": "get_cube_state"}
	getStateJSON, _ := json.Marshal(getStateCmd)
	if err := sendMessage(env.conn, string(getStateJSON)); err != nil {
		return nil, err
	}

	stateMsg, err := receiveMessage(env.conn)
	if err != nil {
		return nil, err
	}
	var stateResp map[string]interface{}
	if err := json.Unmarshal([]byte(stateMsg), &stateResp); err != nil {
		return nil, fmt.Errorf("Error reading initial state: %v", err)
	}

	posArr, _ := parseFloatSlice(stateResp["position"])
	velArr, _ := parseFloatSlice(stateResp["linear_velocity"])

	env.state = append(posArr, velArr...)
	env.prevDist = dist3(env.target, posArr)
	return env.state, nil
}

func (env *GameCubeEnv) Step(action []float64) ([]float64, float64, bool, map[string]float64, error) {
	// send apply_force
	applyCmd := map[string]interface{}{
		"type":  "apply_force",
		"force": action,
	}
	applyJSON, _ := json.Marshal(applyCmd)
	if err := sendMessage(env.conn, string(applyJSON)); err != nil {
		return nil, 0.0, false, nil, err
	}
	time.Sleep(StepWait)

	// get_cube_state
	getStateCmd := map[string]interface{}{"type": "get_cube_state"}
	getStateJSON, _ := json.Marshal(getStateCmd)
	if err := sendMessage(env.conn, string(getStateJSON)); err != nil {
		return nil, 0.0, false, nil, err
	}

	stateMsg, err := receiveMessage(env.conn)
	if err != nil {
		return nil, 0.0, false, nil, err
	}

	var stateResp map[string]interface{}
	if err := json.Unmarshal([]byte(stateMsg), &stateResp); err != nil {
		// fallback
		stateResp = map[string]interface{}{}
	}
	posArr, _ := parseFloatSlice(stateResp["position"])
	velArr, _ := parseFloatSlice(stateResp["linear_velocity"])

	nextState := append(posArr, velArr...)
	currDist := dist3(env.target, posArr)

	improvement := (env.prevDist - currDist) * RewardScale
	reward := improvement - StepPenalty

	// inaction penalty
	if norm(action) < InactionThreshold {
		reward -= InactionPenalty
	}

	done := false
	info := map[string]float64{"distance": currDist}

	if currDist < Tolerance {
		reward += BonusReward
		done = true
	} else if currDist > FarThreshold {
		reward += PenaltyReward
		done = true
		log.Printf("[Env] Over farThreshold => resetting early.")
	}

	env.prevDist = currDist
	env.state = nextState
	return nextState, reward, done, info, nil
}

func (env *GameCubeEnv) Close() {
	env.disconnect()
}

// parseFloatSlice helps parse a JSON array into []float64
func parseFloatSlice(val interface{}) ([]float64, bool) {
	arr, ok := val.([]interface{})
	if !ok {
		return nil, false
	}
	out := make([]float64, len(arr))
	for i, v := range arr {
		f, _ := v.(float64)
		out[i] = f
	}
	return out, true
}

func dist3(target [3]float64, pos []float64) float64 {
	if len(pos) < 3 {
		return 0
	}
	dx := target[0] - pos[0]
	dy := target[1] - pos[1]
	dz := target[2] - pos[2]
	return math.Sqrt(dx*dx + dy*dy + dz*dz)
}

func norm(x []float64) float64 {
	sum := 0.0
	for _, v := range x {
		sum += v * v
	}
	return math.Sqrt(sum)
}

// -----------------------------
// Build Actor/Critic (No Backward calls used)
// -----------------------------
func buildActorNet(stateDim, actionDim, hiddenDim int) *paragon.Network {
	layerSizes := []struct{ Width, Height int }{
		{stateDim, 1},
		{hiddenDim, 1},
		{hiddenDim, 1},
		{actionDim, 1},
	}
	activations := []string{"linear", "relu", "relu", "tanh"}
	fullyConnected := []bool{true, true, true, true}
	net := paragon.NewNetwork(layerSizes, activations, fullyConnected)
	return net
}

func buildCriticNet(stateDim, actionDim, hiddenDim int) *paragon.Network {
	inputDim := stateDim + actionDim
	layerSizes := []struct{ Width, Height int }{
		{inputDim, 1},
		{hiddenDim, 1},
		{hiddenDim, 1},
		{1, 1},
	}
	activations := []string{"linear", "relu", "relu", "linear"}
	fullyConnected := []bool{true, true, true, true}
	net := paragon.NewNetwork(layerSizes, activations, fullyConnected)
	return net
}

// OU noise
func ouNoise(x []float64, mu, theta, sigma float64) []float64 {
	out := make([]float64, len(x))
	for i, v := range x {
		out[i] = v + theta*(mu-v) + sigma*rand.NormFloat64()
	}
	return out
}

func heuristicPolicy(state, target []float64, gain float64) []float64 {
	if len(state) < 3 || len(target) < 3 {
		return []float64{0, 0, 0}
	}
	dx := target[0] - state[0]
	dy := target[1] - state[1]
	dz := target[2] - state[2]
	ax := gain * dx
	ay := gain * dy
	az := gain * dz
	// clamp to [-1..1]
	if ax > 1 {
		ax = 1
	} else if ax < -1 {
		ax = -1
	}
	if ay > 1 {
		ay = 1
	} else if ay < -1 {
		ay = -1
	}
	if az > 1 {
		az = 1
	} else if az < -1 {
		az = -1
	}
	return []float64{ax, ay, az}
}

// -----------------------------
// ReplayBuffer
// -----------------------------
type Transition struct {
	State     []float64
	Action    []float64
	Reward    float64
	NextState []float64
	Done      bool
}

type ReplayBuffer struct {
	capacity int
	storage  []Transition
	mu       sync.Mutex
}

func NewReplayBuffer(cap int) *ReplayBuffer {
	return &ReplayBuffer{
		capacity: cap,
		storage:  make([]Transition, 0, cap),
	}
}

func (rb *ReplayBuffer) Push(st, act []float64, rew float64, nxt []float64, done bool) {
	rb.mu.Lock()
	defer rb.mu.Unlock()
	if len(rb.storage) < rb.capacity {
		rb.storage = append(rb.storage, Transition{st, act, rew, nxt, done})
	} else {
		// Overwrite oldest
		rb.storage[0] = Transition{st, act, rew, nxt, done}
		rb.storage = rb.storage[1:]
		rb.storage = append(rb.storage, Transition{st, act, rew, nxt, done})
	}
}

func (rb *ReplayBuffer) Sample(batchSize int) []Transition {
	rb.mu.Lock()
	defer rb.mu.Unlock()
	if len(rb.storage) < batchSize {
		return nil
	}
	idxes := rand.Perm(len(rb.storage))[:batchSize]
	batch := make([]Transition, batchSize)
	for i, idx := range idxes {
		batch[i] = rb.storage[idx]
	}
	return batch
}

func (rb *ReplayBuffer) Len() int {
	rb.mu.Lock()
	defer rb.mu.Unlock()
	return len(rb.storage)
}

// -----------------------------
// DDPGAgent (No backprop used here, just forward + noise + optional heuristics)
// -----------------------------
type DDPGAgent struct {
	Actor        *paragon.Network
	Critic       *paragon.Network
	TargetActor  *paragon.Network
	TargetCritic *paragon.Network

	// We'll keep references to target so we can do heuristic
	EnvTarget [3]float64
}

func NewDDPGAgent() *DDPGAgent {
	actor := buildActorNet(StateDim, ActionDim, ActorHidden)
	critic := buildCriticNet(StateDim, ActionDim, CriticHidden)

	targetActor := buildActorNet(StateDim, ActionDim, ActorHidden)
	targetCritic := buildCriticNet(StateDim, ActionDim, CriticHidden)

	// copy actor->targetActor
	actor.SaveToJSON("temp_actor.json")
	targetActor.LoadFromJSON("temp_actor.json")

	// copy critic->targetCritic
	critic.SaveToJSON("temp_critic.json")
	targetCritic.LoadFromJSON("temp_critic.json")

	return &DDPGAgent{
		Actor:        actor,
		Critic:       critic,
		TargetActor:  targetActor,
		TargetCritic: targetCritic,
		EnvTarget:    [3]float64{69.58438552780794, 72.56208670917846, 30.29450219308874},
	}
}

// forwardActor
func forwardActorNet(net *paragon.Network, state []float64) []float64 {
	in2d := [][]float64{state}
	net.Forward(in2d)
	outL := net.Layers[net.OutputLayer]
	out := make([]float64, outL.Width)
	for x := 0; x < outL.Width; x++ {
		out[x] = outL.Neurons[0][x].Value
	}
	return out
}

// selectAction: DDPG + optional heuristic/hybrid
func (agent *DDPGAgent) selectAction(state []float64, policyMode string) []float64 {
	// ddpg raw
	ddpgAct := forwardActorNet(agent.Actor, state)
	// add OU noise
	ddpgAct = ouNoise(ddpgAct, 0.0, NoiseTheta, NoiseSigma)

	// clamp to [-1,1], then offset by +2.0 to give more push
	// so final range effectively is [1..3].
	for i := range ddpgAct {
		if ddpgAct[i] > 1 {
			ddpgAct[i] = 1
		} else if ddpgAct[i] < -1 {
			ddpgAct[i] = -1
		}
		ddpgAct[i] += ActionOffset
	}

	// heuristic
	stPos := state[:3]
	hAct := heuristicPolicy(stPos, agent.EnvTarget[:], HeuristicGain)
	// clamp to [-1..1], then offset +2
	for i := range hAct {
		if hAct[i] > 1 {
			hAct[i] = 1
		} else if hAct[i] < -1 {
			hAct[i] = -1
		}
		hAct[i] += ActionOffset
	}

	switch policyMode {
	case "ddpg":
		return ddpgAct
	case "heuristic":
		return hAct
	case "hybrid":
		out := make([]float64, ActionDim)
		for i := 0; i < ActionDim; i++ {
			out[i] = HybridWeight*ddpgAct[i] + (1-HybridWeight)*hAct[i]
		}
		return out
	default:
		return ddpgAct
	}
}

// We'll just skip all gradient updates. The user asked for no backprop.
// We'll still gather transitions + do a "softUpdate" for demonstration,
// but it won't do anything truly meaningful.
func (agent *DDPGAgent) update() {
	// no-op
}

func (agent *DDPGAgent) softUpdate() {
	// no-op
}

// -----------------------------
// trainAgent
// -----------------------------
func trainAgent(agentID, episodes, maxSteps int, policyMode string, results *sync.Map) {
	env := NewGameCubeEnv()
	defer env.Close()

	ddpg := NewDDPGAgent()
	rb := NewReplayBuffer(ReplayCapacity)

	episodeRewards := make([]float64, 0, episodes)

	for ep := 0; ep < episodes; ep++ {
		st, err := env.Reset()
		if err != nil {
			log.Printf("[Agent %d] Reset error: %v", agentID, err)
			episodeRewards = append(episodeRewards, 0)
			continue
		}
		epRew := 0.0

		startTime := time.Now()

		for step := 0; step < maxSteps; step++ {
			if time.Since(startTime) > MaxEpisodeDuration {
				log.Printf("[Agent %d] Episode %d timed out after %v; cutting short.", agentID, ep+1, MaxEpisodeDuration)
				break
			}

			act := ddpg.selectAction(st, policyMode)

			nxt, rew, done, info, err := env.Step(act)
			if err != nil {
				log.Printf("[Agent %d] Step error ep=%d step=%d: %v", agentID, ep+1, step, err)
				break
			}

			epRew += rew
			rb.Push(cloneSlice(st), cloneSlice(act), rew, cloneSlice(nxt), done)

			// extra logs
			if step%10 == 0 {
				log.Printf("[Agent %d] ep=%d step=%d dist=%.2f action=%.2f,%.2f,%.2f rew=%.2f",
					agentID, ep+1, step, info["distance"], act[0], act[1], act[2], rew)
			}

			// skip real DDPG update
			ddpg.update()

			st = nxt
			if done {
				log.Printf("[Agent %d] ep=%d done at step %d with dist=%.2f",
					agentID, ep+1, step, info["distance"])
				break
			}
		}
		episodeRewards = append(episodeRewards, epRew)
		log.Printf("[Agent %d] Episode %d finished with reward %.2f", agentID, ep+1, epRew)
	}

	avg := mean(episodeRewards)
	filename := fmt.Sprintf("ddpg_actor_cube_agent_%d.json", agentID)
	// Just save the actor, though it doesn't actually train
	if err := ddpg.Actor.SaveToJSON(filename); err != nil {
		log.Printf("[Agent %d] Could not save actor model: %v\n", agentID, err)
	}
	results.Store(agentID, struct {
		AvgReward float64
		ModelFile string
	}{AvgReward: avg, ModelFile: filename})
}

func mean(data []float64) float64 {
	if len(data) == 0 {
		return 0
	}
	total := 0.0
	for _, v := range data {
		total += v
	}
	return total / float64(len(data))
}

// cloneSlice
func cloneSlice(arr []float64) []float64 {
	c := make([]float64, len(arr))
	copy(c, arr)
	return c
}

func main() {
	rand.Seed(time.Now().UnixNano())

	var wg sync.WaitGroup
	results := sync.Map{}

	for i := 1; i <= NumAgents; i++ {
		wg.Add(1)
		agentID := i
		go func() {
			defer wg.Done()
			trainAgent(agentID, Episodes, MaxSteps, PolicyMode, &results)
		}()
	}
	wg.Wait()

	// choose best
	var bestAgent int
	bestAvg := math.Inf(-1)
	var bestFile string

	results.Range(func(key, value interface{}) bool {
		agentID := key.(int)
		data := value.(struct {
			AvgReward float64
			ModelFile string
		})
		fmt.Printf("Agent %d: Average Reward = %.2f, ModelFile=%s\n",
			agentID, data.AvgReward, data.ModelFile)
		if data.AvgReward > bestAvg {
			bestAvg = data.AvgReward
			bestAgent = agentID
			bestFile = data.ModelFile
		}
		return true
	})

	fmt.Printf("Best agent is Agent %d with average reward %.2f.\n", bestAgent, bestAvg)
	if bestFile != "" {
		// rename or copy to "best"
		err := os.Rename(bestFile, "ddpg_actor_cube_best.json")
		if err != nil {
			fmt.Printf("Could not rename best model: %v\n", err)
		} else {
			fmt.Println("Best agent model saved as 'ddpg_actor_cube_best.json'.")
		}
	}
}
