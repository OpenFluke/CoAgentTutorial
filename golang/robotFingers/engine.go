package main

import (
	"bytes"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"net"
	"sort"
	"strings"
	"time"
)

// --------------------
// Server connection info
// --------------------
const (
	ServerIP      = "127.0.0.1"
	ServerPort    = 14000
	SharedPass    = "my_secure_password"
	EndOfMsg      = "<???DONE???---"
	SocketTimeout = 2 * time.Second
)

// --------------------
// We'll define a struct to track our hand's cubes/joints
// --------------------
type GiantHand struct {
	conn       net.Conn
	Palm       string     // The frozen base "palm" cube
	Fingers    [][]string // Each finger is a list of segment cube names
	HingeNames [][]string // Hinge joint names for each finger
}

func NewGiantHand(conn net.Conn) *GiantHand {
	return &GiantHand{
		conn:       conn,
		Palm:       "",
		Fingers:    [][]string{},
		HingeNames: [][]string{},
	}
}

// --------------------
// MAIN
// --------------------
func main() {
	// 1) Connect & Authenticate
	conn, err := net.Dial("tcp", fmt.Sprintf("%s:%d", ServerIP, ServerPort))
	if err != nil {
		fmt.Printf("Failed to connect to server: %v\n", err)
		return
	}
	defer conn.Close()

	if err := sendMessage(conn, SharedPass); err != nil {
		fmt.Printf("Auth send error: %v\n", err)
		return
	}
	authResp, err := receiveMessage(conn)
	if err != nil {
		fmt.Printf("Auth receive error: %v\n", err)
		return
	}
	if authResp != "auth_success" {
		fmt.Printf("Authentication failed: %s\n", authResp)
		return
	}
	fmt.Println("✅ Authenticated with the server.")

	// 2) Create the GiantHand
	hand := NewGiantHand(conn)
	if err := hand.BuildHand(); err != nil {
		fmt.Printf("BuildHand error: %v\n", err)
		return
	}

	// 3) Make the hand wave
	hand.WaveHand()

	// 4) Despawn everything
	hand.Cleanup()
	fmt.Println("All done! Exiting now.")
}

// BuildHand spawns one frozen "palm" and multiple "fingers" each with a few segments.
func (gh *GiantHand) BuildHand() error {
	fmt.Println("=== Spawning Giant Hand ===")

	// 1) Spawn palm (frozen base)
	palmPos := []float64{0, 100, 40}
	palmCube, err := spawnCube(gh.conn, palmPos, []float64{0, 0, 0}, true)
	if err != nil {
		return fmt.Errorf("failed to spawn palm: %v", err)
	}
	gh.Palm = palmCube

	// 2) Let’s define a few "fingers". E.g. 5 fingers, each with 3 segments (like distal, middle, proximal).
	fingerCount := 5
	segmentsPerFinger := 3

	// We'll position each finger along +Z direction from the palm
	// Then each segment along +X direction from the "knuckle"
	// so that each finger has hinge joints that can wave up/down
	for f := 0; f < fingerCount; f++ {
		var fingerCubes []string
		var fingerHinges []string

		// Position of the knuckle base on the palm
		// We offset in Z by (f - 2)*1.5, so fingers spread out
		knucklePos := []float64{
			palmPos[0],
			palmPos[1],
			palmPos[2] + float64(f-2)*1.5,
		}

		// Spawn the "knuckle" cube for this finger (hinge to the palm)
		cName, err := spawnCube(gh.conn, knucklePos, []float64{0, 0, 0}, false)
		if err != nil {
			return fmt.Errorf("finger %d knuckle spawn error: %v", f, err)
		}
		fingerCubes = append(fingerCubes, cName)

		// Create a hinge from the palm to this knuckle
		anchor := midPoint(knucklePos, palmPos)
		jName := createJointAt(gh.conn, gh.Palm, cName, "hinge", anchor)
		fingerHinges = append(fingerHinges, jName)

		// Now for each additional segment
		segOffsetX := 1.0 // each finger segment is 1 unit along +X
		for s := 1; s < segmentsPerFinger; s++ {
			segPos := []float64{
				knucklePos[0] + float64(s)*segOffsetX,
				knucklePos[1],
				knucklePos[2],
			}
			segCube, err := spawnCube(gh.conn, segPos, []float64{0, 0, 0}, false)
			if err != nil {
				return fmt.Errorf("finger %d segment %d spawn error: %v", f, s, err)
			}
			fingerCubes = append(fingerCubes, segCube)

			// hinge from the previous segment
			anchor2 := midPoint(segPos, []float64{
				knucklePos[0] + float64(s-1)*segOffsetX,
				knucklePos[1],
				knucklePos[2],
			})
			prevCube := fingerCubes[len(fingerCubes)-2] // the last one
			segJoint := createJointAt(gh.conn, prevCube, segCube, "hinge", anchor2)
			fingerHinges = append(fingerHinges, segJoint)
		}

		// store this finger's cubes & hinge names
		gh.Fingers = append(gh.Fingers, fingerCubes)
		gh.HingeNames = append(gh.HingeNames, fingerHinges)
	}

	// That’s it. We have a palm plus finger segments each hinged.
	fmt.Println("Hand built successfully.")
	return nil
}

// WaveHand swings each finger back and forth by enabling motor on each hinge, then reversing.
func (gh *GiantHand) WaveHand() {
	fmt.Println("=== Waving the hand ===")

	// We'll do a few cycles. In each cycle, we:
	// 1) rotate each finger's hinges forward
	// 2) wait a bit
	// 3) rotate them backward
	// 4) wait
	// Then stop motors.

	cycles := 2
	forwardVel := 1.5
	backwardVel := -1.5
	maxImpulse := 5.0
	waitTime := 1 * time.Second

	for c := 0; c < cycles; c++ {
		// Step 1) forward
		for f := range gh.HingeNames {
			for _, jName := range gh.HingeNames[f] {
				if jName == "" {
					continue
				}
				setJointParam(gh.conn, jName, "motor_target_velocity", forwardVel)
				setJointParam(gh.conn, jName, "motor_max_impulse", maxImpulse)
			}
		}
		time.Sleep(waitTime)

		// Step 2) reverse
		for f := range gh.HingeNames {
			for _, jName := range gh.HingeNames[f] {
				if jName == "" {
					continue
				}
				setJointParam(gh.conn, jName, "motor_target_velocity", backwardVel)
			}
		}
		time.Sleep(waitTime)
	}

	// Finally stop all motors
	for f := range gh.HingeNames {
		for _, jName := range gh.HingeNames[f] {
			setJointParam(gh.conn, jName, "motor_target_velocity", 0.0)
		}
	}

	fmt.Println("=== Done waving ===")
}

// Cleanup despawns all cubes (palm + all fingers).
func (gh *GiantHand) Cleanup() {
	fmt.Println("=== Cleaning up the giant hand ===")

	// Despawn finger cubes first
	for _, fingerCubes := range gh.Fingers {
		for _, c := range fingerCubes {
			if c != "" {
				despawnCube(gh.conn, c)
			}
		}
	}

	// Despawn palm last
	if gh.Palm != "" {
		despawnCube(gh.conn, gh.Palm)
	}

	fmt.Println("All cubes removed.")
}

// -------------------------------------------------------------------
// Below are the same helper functions (spawnCube, createJointAt, etc.)
// that you had in your existing code. Only the logic changed to build
// a “hand” that waves instead of an arm/claw.
// -------------------------------------------------------------------

type CubeListResponse struct {
	Type  string   `json:"type"`
	Cubes []string `json:"cubes"`
}
type JointCreatedResponse struct {
	Type      string `json:"type"`
	JointName string `json:"joint_name"`
}

// sendMessage: appends EndOfMsg and writes to the connection
func sendMessage(conn net.Conn, message string) error {
	fullMsg := message + EndOfMsg
	_, err := conn.Write([]byte(fullMsg))
	return err
}

// receiveMessage: reads until EndOfMsg or we time out
func receiveMessage(conn net.Conn) (string, error) {
	conn.SetReadDeadline(time.Now().Add(SocketTimeout))

	var buffer bytes.Buffer
	tmp := make([]byte, 1024)

	for {
		n, err := conn.Read(tmp)
		if err != nil {
			if err == io.EOF {
				break
			}
			return buffer.String(), err
		}
		buffer.Write(tmp[:n])
		if bytes.Contains(buffer.Bytes(), []byte(EndOfMsg)) {
			break
		}
	}
	str := buffer.String()
	if idx := strings.Index(str, EndOfMsg); idx >= 0 {
		str = str[:idx]
	}
	return strings.TrimSpace(str), nil
}

// getCubeList requests the current list of cubes from the server.
func getCubeList(conn net.Conn) (CubeListResponse, error) {
	var res CubeListResponse
	cmd := map[string]interface{}{"type": "get_cube_list"}
	data, _ := json.Marshal(cmd)
	if err := sendMessage(conn, string(data)); err != nil {
		return res, err
	}
	resp, err := receiveMessage(conn)
	if err != nil {
		return res, err
	}
	if err := json.Unmarshal([]byte(resp), &res); err != nil {
		return res, fmt.Errorf("unmarshal get_cube_list error: %v", err)
	}
	if res.Type != "cube_list" {
		return res, fmt.Errorf("unexpected response: %s", resp)
	}
	return res, nil
}

// spawnCube spawns a single cube, optionally isBase => frozen in place
func spawnCube(conn net.Conn, position, rotation []float64, isBase bool) (string, error) {
	oldList, err := getCubeList(conn)
	if err != nil {
		return "", fmt.Errorf("get old list error: %v", err)
	}
	oldCubes := append([]string(nil), oldList.Cubes...)

	cmd := map[string]interface{}{
		"type":     "spawn_cube",
		"position": position,
		"rotation": rotation,
		"is_base":  isBase,
	}
	data, _ := json.Marshal(cmd)
	if err := sendMessage(conn, string(data)); err != nil {
		return "", fmt.Errorf("send spawn_cube error: %v", err)
	}

	// Poll for a new cube that wasn't in oldList
	var newCubeName string
	for i := 0; i < 20; i++ {
		time.Sleep(200 * time.Millisecond)
		newList, err := getCubeList(conn)
		if err != nil {
			return "", err
		}
		diff := difference(newList.Cubes, oldCubes)
		if len(diff) == 1 {
			newCubeName = diff[0]
			break
		} else if len(diff) > 1 {
			return "", fmt.Errorf("found multiple new cubes: %v", diff)
		}
	}
	if newCubeName == "" {
		return "", errors.New("spawnCube timed out; no new cube found")
	}
	fmt.Printf("[spawnCube] Created: %s at %v (isBase=%v)\n", newCubeName, position, isBase)
	return newCubeName, nil
}

// createJointAt: link two cubes with a certain joint type at a chosen anchor.
func createJointAt(conn net.Conn, cubeA, cubeB, jointType string, anchor []float64) string {
	cmd := map[string]interface{}{
		"type":       "create_joint",
		"cube1":      cubeA,
		"cube2":      cubeB,
		"joint_type": jointType,
		"position":   anchor,
	}
	data, _ := json.Marshal(cmd)
	if err := sendMessage(conn, string(data)); err != nil {
		fmt.Printf("createJointAt send error: %v\n", err)
		return ""
	}
	resp, err := receiveMessage(conn)
	if err != nil {
		fmt.Printf("createJointAt recv error: %v\n", err)
		return ""
	}
	var jres JointCreatedResponse
	if err := json.Unmarshal([]byte(resp), &jres); err != nil {
		fmt.Printf("createJointAt parse error: %v\n", err)
		return ""
	}
	if jres.Type != "joint_created" {
		fmt.Printf("Unexpected create_joint response: %s\n", resp)
		return ""
	}
	fmt.Printf("[createJointAt] Created joint '%s' for %s -> %s\n", jres.JointName, cubeA, cubeB)
	return jres.JointName
}

// setJointParam: sets e.g. "motor_target_velocity" or "motor_max_impulse" on a hinge.
func setJointParam(conn net.Conn, jointName, paramName string, value float64) {
	if jointName == "" {
		return
	}
	cmd := map[string]interface{}{
		"type":       "set_joint_param",
		"joint_name": jointName,
		"param_name": paramName,
		"value":      value,
	}
	data, _ := json.Marshal(cmd)
	if err := sendMessage(conn, string(data)); err != nil {
		fmt.Printf("setJointParam error: %v\n", err)
		return
	}
	resp, _ := receiveMessage(conn)
	fmt.Printf("[setJointParam] %s param=%s => %.2f; resp=%s\n", jointName, paramName, value, resp)
}

// despawnCube: remove a named cube
func despawnCube(conn net.Conn, cName string) {
	cmd := map[string]interface{}{
		"type":      "despawn_cube",
		"cube_name": cName,
	}
	data, _ := json.Marshal(cmd)
	if err := sendMessage(conn, string(data)); err != nil {
		fmt.Printf("despawnCube send error: %v\n", err)
		return
	}
	resp, _ := receiveMessage(conn)
	fmt.Printf("[despawnCube] %s => %s\n", cName, resp)
}

// difference: returns items in A not in B
func difference(a, b []string) []string {
	mb := make(map[string]bool, len(b))
	for _, x := range b {
		mb[x] = true
	}
	var diff []string
	for _, x := range a {
		if !mb[x] {
			diff = append(diff, x)
		}
	}
	sort.Strings(diff)
	return diff
}

// midPoint: average of two positions
func midPoint(a, b []float64) []float64 {
	return []float64{
		(a[0] + b[0]) * 0.5,
		(a[1] + b[1]) * 0.5,
		(a[2] + b[2]) * 0.5,
	}
}
