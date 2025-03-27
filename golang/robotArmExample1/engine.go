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

const (
	ServerIP      = "192.168.0.228"
	ServerPort    = 14000
	SharedPass    = "my_secure_password"
	EndOfMsg      = "<???DONE???---" // Must match the Godot server's marker
	SocketTimeout = 2 * time.Second
)

// RobotArmWithClaw holds references to each spawned cube & joint for easy cleanup.
type RobotArmWithClaw struct {
	conn net.Conn

	// Main arm cubes (like a chain)
	ArmCubes  []string
	ArmJoints []string

	// Claw cubes
	ClawTopCube    string
	ClawBottomCube string
	ClawJoints     []string

	// A target cube to pick up
	TargetCube string
}

// NewRobotArmWithClaw initializes our structure.
func NewRobotArmWithClaw(conn net.Conn) *RobotArmWithClaw {
	return &RobotArmWithClaw{conn: conn}
}

// BuildArm spawns a base cube (frozen) plus a series of “chain” cubes to form an arm.
// Then it creates hinge joints between consecutive cubes.
func (arm *RobotArmWithClaw) BuildArm(numSegments int) error {
	fmt.Println("=== Building Robot Arm ===")

	// Higher base position so the arm is elevated.
	startPos := []float64{0, 110, 40}
	offset := []float64{1, 0, 0} // each additional segment is +1 in X

	// 1) Spawn the base (first) cube; mark it as static.
	baseCube, err := spawnCube(arm.conn, startPos, []float64{0, 0, 0}, true)
	if err != nil {
		return fmt.Errorf("failed to spawn base cube: %v", err)
	}
	arm.ArmCubes = append(arm.ArmCubes, baseCube)

	// 2) Spawn additional segments.
	for i := 1; i < numSegments; i++ {
		pos := []float64{
			startPos[0] + float64(i)*offset[0],
			startPos[1] + float64(i)*offset[1],
			startPos[2] + float64(i)*offset[2],
		}
		cName, err := spawnCube(arm.conn, pos, []float64{0, 0, 0}, false)
		if err != nil {
			return fmt.Errorf("failed to spawn segment %d: %v", i, err)
		}
		arm.ArmCubes = append(arm.ArmCubes, cName)
	}

	// 3) Create hinge joints between consecutive cubes.
	for i := 0; i < len(arm.ArmCubes)-1; i++ {
		cubeA := arm.ArmCubes[i]
		cubeB := arm.ArmCubes[i+1]

		// Anchor roughly halfway between A and B.
		anchorPos := []float64{
			startPos[0] + float64(i)*offset[0] + 0.5*offset[0],
			startPos[1] + float64(i)*offset[1] + 0.5*offset[1],
			startPos[2] + float64(i)*offset[2] + 0.5*offset[2],
		}

		jName := createJointAt(arm.conn, cubeA, cubeB, "hinge", anchorPos)
		arm.ArmJoints = append(arm.ArmJoints, jName)
	}

	fmt.Printf("Built arm with %d cubes, %d joints.\n", len(arm.ArmCubes), len(arm.ArmJoints))
	return nil
}

// AddClaw spawns two extra cubes (“clawTop” and “clawBottom”), attaching them to the last arm segment.
func (arm *RobotArmWithClaw) AddClaw() error {
	if len(arm.ArmCubes) == 0 {
		return errors.New("arm has no cubes; build the arm first")
	}
	lastCube := arm.ArmCubes[len(arm.ArmCubes)-1]

	// We'll spawn the claw cubes near the last segment’s position.
	// Offset them slightly up and down.
	topOffset := []float64{0, 1, 0}
	bottomOffset := []float64{0, -1, 0}

	// Get the last segment's position from the server.
	lastPos, err := getCubePosition(arm.conn, lastCube)
	if err != nil {
		return fmt.Errorf("could not fetch lastCube position: %v", err)
	}

	topPos := []float64{
		lastPos[0] + topOffset[0],
		lastPos[1] + topOffset[1],
		lastPos[2] + topOffset[2],
	}
	botPos := []float64{
		lastPos[0] + bottomOffset[0],
		lastPos[1] + bottomOffset[1],
		lastPos[2] + bottomOffset[2],
	}

	clawTop, err := spawnCube(arm.conn, topPos, []float64{0, 0, 0}, false)
	if err != nil {
		return fmt.Errorf("failed to spawn top claw cube: %v", err)
	}
	clawBottom, err := spawnCube(arm.conn, botPos, []float64{0, 0, 0}, false)
	if err != nil {
		return fmt.Errorf("failed to spawn bottom claw cube: %v", err)
	}
	arm.ClawTopCube = clawTop
	arm.ClawBottomCube = clawBottom

	// Attach each claw cube to the last segment with pin joints.
	topAnchor := midPoint(topPos, lastPos)
	botAnchor := midPoint(botPos, lastPos)

	topJoint := createJointAt(arm.conn, lastCube, clawTop, "pin", topAnchor)
	botJoint := createJointAt(arm.conn, lastCube, clawBottom, "pin", botAnchor)
	arm.ClawJoints = append(arm.ClawJoints, topJoint, botJoint)

	fmt.Printf("Claw added: top=%s bottom=%s\n", clawTop, clawBottom)
	return nil
}

// SpawnTarget places a separate cube in front of the arm so we can pick it up.
func (arm *RobotArmWithClaw) SpawnTarget() error {
	if len(arm.ArmCubes) == 0 {
		return errors.New("cannot spawn target; arm has no cubes")
	}
	lastCube := arm.ArmCubes[len(arm.ArmCubes)-1]

	lastPos, err := getCubePosition(arm.conn, lastCube)
	if err != nil {
		return fmt.Errorf("failed to get last segment position: %v", err)
	}
	// Place the target 5 units further along +X from the last segment.
	targetPos := []float64{lastPos[0] + 5, lastPos[1], lastPos[2]}
	tName, err := spawnCube(arm.conn, targetPos, []float64{0, 0, 0}, false)
	if err != nil {
		return fmt.Errorf("failed to spawn target: %v", err)
	}
	arm.TargetCube = tName
	fmt.Printf("Target spawned at %v -> %s\n", targetPos, tName)
	return nil
}

// PickUpTarget demonstrates simple movement of the arm and then "grabs" the target by pinning it.
func (arm *RobotArmWithClaw) PickUpTarget() {
	fmt.Println("=== Attempting to pick up target ===")
	if len(arm.ArmJoints) == 0 || arm.TargetCube == "" {
		fmt.Println("No joints or no target, cannot pick up.")
		return
	}
	// Move the first joint a bit.
	firstJoint := arm.ArmJoints[0]
	setJointParam(arm.conn, firstJoint, "motor_target_velocity", 1.0)
	setJointParam(arm.conn, firstJoint, "motor_max_impulse", 8.0)
	time.Sleep(2 * time.Second)
	// Reverse direction.
	setJointParam(arm.conn, firstJoint, "motor_target_velocity", -1.0)
	time.Sleep(2 * time.Second)
	// Stop the motor.
	setJointParam(arm.conn, firstJoint, "motor_target_velocity", 0.0)

	// Now "grab" the target by pinning it to the claw top.
	topPos, err := getCubePosition(arm.conn, arm.ClawTopCube)
	if err != nil {
		fmt.Printf("Could not get top claw position: %v\n", err)
		return
	}
	targetPos, err := getCubePosition(arm.conn, arm.TargetCube)
	if err != nil {
		fmt.Printf("Could not get target position: %v\n", err)
		return
	}
	anchor := midPoint(topPos, targetPos)
	grabJoint := createJointAt(arm.conn, arm.ClawTopCube, arm.TargetCube, "pin", anchor)
	arm.ClawJoints = append(arm.ClawJoints, grabJoint)
	fmt.Println("Target pinned to claw top. Now lifting...")

	// Lift the arm by rotating the second joint, if available.
	if len(arm.ArmJoints) > 1 {
		secondJoint := arm.ArmJoints[1]
		setJointParam(arm.conn, secondJoint, "motor_target_velocity", -1.5)
		setJointParam(arm.conn, secondJoint, "motor_max_impulse", 8.0)
		time.Sleep(3 * time.Second)
		setJointParam(arm.conn, secondJoint, "motor_target_velocity", 0.0)
	} else {
		fmt.Println("No second joint to lift with, skipping lift.")
	}

	fmt.Println("Target pick-up sequence complete.")
}

// Dismantle removes all cubes (arm, claw, target) from the scene.
func (arm *RobotArmWithClaw) Dismantle() {
	fmt.Println("=== Dismantling entire setup ===")
	// Despawn target first.
	if arm.TargetCube != "" {
		despawnCube(arm.conn, arm.TargetCube)
		arm.TargetCube = ""
	}
	// Despawn claw cubes.
	if arm.ClawTopCube != "" {
		despawnCube(arm.conn, arm.ClawTopCube)
	}
	if arm.ClawBottomCube != "" {
		despawnCube(arm.conn, arm.ClawBottomCube)
	}
	arm.ClawTopCube = ""
	arm.ClawBottomCube = ""
	// Despawn arm cubes.
	for _, c := range arm.ArmCubes {
		despawnCube(arm.conn, c)
	}
	arm.ArmCubes = nil
	arm.ArmJoints = nil
	arm.ClawJoints = nil
	fmt.Println("All cubes despawned.")
}

// -----------------------------
// Main Function
// -----------------------------
func main() {
	conn, err := net.Dial("tcp", fmt.Sprintf("%s:%d", ServerIP, ServerPort))
	if err != nil {
		fmt.Printf("Failed to connect to server: %v\n", err)
		return
	}
	defer conn.Close()

	// 1) Authenticate.
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
	fmt.Println("✅ Authenticated with server.")

	// 2) Build the arm with claw.
	arm := NewRobotArmWithClaw(conn)
	if err := arm.BuildArm(8); err != nil {
		fmt.Printf("Error building arm: %v\n", err)
		return
	}
	if err := arm.AddClaw(); err != nil {
		fmt.Printf("Error adding claw: %v\n", err)
		return
	}

	// 3) Spawn a target cube to pick up.
	if err := arm.SpawnTarget(); err != nil {
		fmt.Printf("Error spawning target: %v\n", err)
		return
	}

	// 4) Attempt to pick up the target.
	arm.PickUpTarget()

	// 5) Cleanup.
	arm.Dismantle()
	fmt.Println("✅ Demo complete. Exiting.")
}

// -----------------------------
// Networking / Helper Functions
// -----------------------------

func sendMessage(conn net.Conn, message string) error {
	msg := message + EndOfMsg
	_, err := conn.Write([]byte(msg))
	return err
}

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

type CubeListResponse struct {
	Type  string   `json:"type"`
	Cubes []string `json:"cubes"`
}

// getCubeList requests the server's current list of cubes.
func getCubeList(conn net.Conn) (CubeListResponse, error) {
	var result CubeListResponse
	cmd := map[string]interface{}{"type": "get_cube_list"}
	out, _ := json.Marshal(cmd)
	if err := sendMessage(conn, string(out)); err != nil {
		return result, err
	}
	resp, err := receiveMessage(conn)
	if err != nil {
		return result, err
	}
	if err := json.Unmarshal([]byte(resp), &result); err != nil {
		return result, fmt.Errorf("unmarshal get_cube_list: %v", err)
	}
	if result.Type != "cube_list" {
		return result, fmt.Errorf("invalid get_cube_list response: %s", resp)
	}
	return result, nil
}

// spawnCube requests the server to spawn a single cube at the given position & rotation.
func spawnCube(conn net.Conn, position, rotation []float64, isBase bool) (string, error) {
	oldList, err := getCubeList(conn)
	if err != nil {
		return "", fmt.Errorf("failed to get old cube list: %v", err)
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

	var newCubeName string
	const maxPoll = 20
	for i := 0; i < maxPoll; i++ {
		time.Sleep(200 * time.Millisecond)
		newList, err := getCubeList(conn)
		if err != nil {
			return "", err
		}
		newCubes := difference(newList.Cubes, oldCubes)
		if len(newCubes) == 1 {
			newCubeName = newCubes[0]
			break
		} else if len(newCubes) > 1 {
			return "", fmt.Errorf("unexpectedly found %d new cubes: %v", len(newCubes), newCubes)
		}
	}
	if newCubeName == "" {
		return "", errors.New("spawn_cube timed out: no new cube found")
	}
	fmt.Printf("[spawnCube] Created: %s at %v\n", newCubeName, position)
	return newCubeName, nil
}

type JointCreatedResponse struct {
	Type      string `json:"type"`
	JointName string `json:"joint_name"`
}

// createJointAt links two cubes (cubeA, cubeB) with a joint of jointType at the given anchor.
func createJointAt(conn net.Conn, cubeA, cubeB, jointType string, anchor []float64) string {
	cmd := map[string]interface{}{
		"type":       "create_joint",
		"cube1":      cubeA,
		"cube2":      cubeB,
		"joint_type": jointType,
		"position":   anchor,
	}
	out, _ := json.Marshal(cmd)
	if err := sendMessage(conn, string(out)); err != nil {
		fmt.Printf("create_joint error: %v\n", err)
		return ""
	}
	resp, err := receiveMessage(conn)
	if err != nil {
		fmt.Printf("create_joint read error: %v\n", err)
		return ""
	}
	var jr JointCreatedResponse
	if err := json.Unmarshal([]byte(resp), &jr); err != nil {
		fmt.Printf("createJointAt parse error: %v\n", err)
		return ""
	}
	if jr.Type != "joint_created" {
		fmt.Printf("Unexpected create_joint response: %s\n", resp)
		return ""
	}
	fmt.Printf("[createJointAt] Created '%s' between %s and %s\n", jr.JointName, cubeA, cubeB)
	return jr.JointName
}

// setJointParam adjusts a physics parameter (e.g. "motor_target_velocity") on a joint.
func setJointParam(conn net.Conn, jointName, paramName string, value float64) {
	cmd := map[string]interface{}{
		"type":       "set_joint_param",
		"joint_name": jointName,
		"param_name": paramName,
		"value":      value,
	}
	out, _ := json.Marshal(cmd)
	if err := sendMessage(conn, string(out)); err != nil {
		fmt.Printf("setJointParam send error: %v\n", err)
		return
	}
	resp, _ := receiveMessage(conn)
	fmt.Printf("[setJointParam] joint=%s param=%s => %v | resp=%s\n", jointName, paramName, value, resp)
}

// despawnCube tells the server to remove a named cube from the scene.
func despawnCube(conn net.Conn, cubeName string) {
	cmd := map[string]interface{}{
		"type":      "despawn_cube",
		"cube_name": cubeName,
	}
	out, _ := json.Marshal(cmd)
	if err := sendMessage(conn, string(out)); err != nil {
		fmt.Printf("despawnCube send error: %v\n", err)
		return
	}
	resp, _ := receiveMessage(conn)
	fmt.Printf("[despawnCube] Removed %s -> resp=%s\n", cubeName, resp)
}

// difference returns elements in sliceA that are not in sliceB.
func difference(sliceA, sliceB []string) []string {
	mb := make(map[string]bool, len(sliceB))
	for _, x := range sliceB {
		mb[x] = true
	}
	var diff []string
	for _, x := range sliceA {
		if !mb[x] {
			diff = append(diff, x)
		}
	}
	sort.Strings(diff)
	return diff
}

// midPoint calculates the midpoint between two positions (as slices).
func midPoint(a, b []float64) []float64 {
	return []float64{
		(a[0] + b[0]) * 0.5,
		(a[1] + b[1]) * 0.5,
		(a[2] + b[2]) * 0.5,
	}
}

// getCubePosition calls "get_cube_info" to fetch the current position of a named cube.
func getCubePosition(conn net.Conn, cubeName string) ([]float64, error) {
	if cubeName == "" {
		return nil, errors.New("cubeName is empty")
	}
	cmd := map[string]interface{}{
		"type":      "get_cube_info",
		"cube_name": cubeName,
	}
	out, _ := json.Marshal(cmd)
	if err := sendMessage(conn, string(out)); err != nil {
		return nil, err
	}
	resp, err := receiveMessage(conn)
	if err != nil {
		return nil, err
	}
	// Expecting JSON with "position": [x, y, z]
	var info map[string]interface{}
	if err := json.Unmarshal([]byte(resp), &info); err != nil {
		return nil, fmt.Errorf("json parse error: %v", err)
	}
	if info["type"] != "cube_info" {
		return nil, fmt.Errorf("invalid response for get_cube_info: %s", resp)
	}
	posVal, ok := info["position"].([]interface{})
	if !ok || len(posVal) != 3 {
		return nil, fmt.Errorf("invalid or missing position array in response: %s", resp)
	}
	result := make([]float64, 3)
	for i := 0; i < 3; i++ {
		f, ok := posVal[i].(float64)
		if !ok {
			return nil, fmt.Errorf("position[%d] not a float: %v", i, posVal[i])
		}
		result[i] = f
	}
	return result, nil
}
