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
	ServerIP      = "127.0.0.1"
	ServerPort    = 14000
	SharedPass    = "my_secure_password"
	EndOfMsg      = "<???DONE???---" // Must match the Godot server's marker
	SocketTimeout = 2 * time.Second
)

// CubeGrid holds references to cubes spawned in a grid as well as the attached pole.
type CubeGrid struct {
	conn       net.Conn
	GridCubes  [][]string // 2D slice storing cube names for each grid cell.
	PoleCubes  []string   // List of cube names for the vertical pole (attached to center).
	GridJoints []string   // Joints connecting grid cubes.
	PoleJoints []string   // Joints connecting pole cubes.
}

// BuildGrid spawns a grid of cubes on the XZ plane, offset so that they are on top of the planet.
// The grid is centered around a given base position (baseX, baseY, baseZ) so that cubes appear elevated.
func (cg *CubeGrid) BuildGrid(rows, cols int, spacing float64) error {
	fmt.Println("=== Building Cube Grid ===")
	// Base position offset so that cubes are on top of the planet.
	baseX, baseY, baseZ := 20.0, 100.0, 0.0

	// Allocate grid slice.
	cg.GridCubes = make([][]string, rows)
	for i := range cg.GridCubes {
		cg.GridCubes[i] = make([]string, cols)
	}

	// Determine starting position so that the grid is centered at (baseX, baseY, baseZ) on the XZ plane.
	startX := baseX - float64(cols-1)*spacing/2.0
	startZ := baseZ - float64(rows-1)*spacing/2.0
	y := baseY

	// Spawn each cube in the grid.
	for i := 0; i < rows; i++ {
		for j := 0; j < cols; j++ {
			pos := []float64{
				startX + float64(j)*spacing,
				y,
				startZ + float64(i)*spacing,
			}
			cubeName, err := spawnCube(cg.conn, pos, []float64{0, 0, 0}, true)
			if err != nil {
				return fmt.Errorf("failed to spawn cube at (%d,%d): %v", i, j, err)
			}
			cg.GridCubes[i][j] = cubeName

			// Connect with left neighbor if it exists.
			if j > 0 {
				leftCube := cg.GridCubes[i][j-1]
				leftPos := []float64{
					startX + float64(j-1)*spacing,
					y,
					startZ + float64(i)*spacing,
				}
				anchor := midPoint(pos, leftPos)
				jointName := createJointAt(cg.conn, leftCube, cubeName, "hinge", anchor)
				cg.GridJoints = append(cg.GridJoints, jointName)
			}
			// Connect with top neighbor if it exists.
			if i > 0 {
				topCube := cg.GridCubes[i-1][j]
				topPos := []float64{
					startX + float64(j)*spacing,
					y,
					startZ + float64(i-1)*spacing,
				}
				anchor := midPoint(pos, topPos)
				jointName := createJointAt(cg.conn, topCube, cubeName, "hinge", anchor)
				cg.GridJoints = append(cg.GridJoints, jointName)
			}
		}
	}
	fmt.Printf("Grid built with %d rows and %d cols (%d cubes total).\n", rows, cols, rows*cols)
	return nil
}

// AddPole attaches a vertical column (pole) of cubes to the center cube of the grid.
// It spawns numPole cubes above the center and connects them using pin joints.
func (cg *CubeGrid) AddPole(numPole int, verticalSpacing float64) error {
	fmt.Println("=== Attaching Pole to Grid Center ===")
	// Find the center cube of the grid.
	rows := len(cg.GridCubes)
	if rows == 0 {
		return errors.New("grid has no rows")
	}
	cols := len(cg.GridCubes[0])
	centerRow, centerCol := rows/2, cols/2
	centerCube := cg.GridCubes[centerRow][centerCol]

	// Get the center cube's current position.
	centerPos, err := getCubePosition(cg.conn, centerCube)
	if err != nil {
		return fmt.Errorf("failed to get center cube position: %v", err)
	}

	// Build the pole upward.
	prevCube := centerCube
	for i := 1; i <= numPole; i++ {
		// New position: raise Y by verticalSpacing each time.
		newPos := []float64{
			centerPos[0],
			centerPos[1] + float64(i)*verticalSpacing,
			centerPos[2],
		}
		cubeName, err := spawnCube(cg.conn, newPos, []float64{0, 0, 0}, false)
		if err != nil {
			return fmt.Errorf("failed to spawn pole cube %d: %v", i, err)
		}
		cg.PoleCubes = append(cg.PoleCubes, cubeName)
		anchor := midPoint(newPos, centerPos) // simple anchor between center and new cube
		jointName := createJointAt(cg.conn, prevCube, cubeName, "pin", anchor)
		cg.PoleJoints = append(cg.PoleJoints, jointName)
		// Update centerPos for next cube in the pole.
		centerPos = newPos
		prevCube = cubeName
	}
	fmt.Printf("Pole attached with %d cubes above the center cube.\n", numPole)
	return nil
}

// Dismantle despawns all cubes (both grid and pole) from the scene.
func (cg *CubeGrid) Dismantle() {
	fmt.Println("=== Dismantling Cube Grid and Pole ===")
	// Despawn pole cubes.
	for _, cube := range cg.PoleCubes {
		despawnCube(cg.conn, cube)
	}
	cg.PoleCubes = nil
	cg.PoleJoints = nil

	// Despawn grid cubes.
	for i := range cg.GridCubes {
		for j := range cg.GridCubes[i] {
			despawnCube(cg.conn, cg.GridCubes[i][j])
		}
	}
	cg.GridCubes = nil
	cg.GridJoints = nil
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

	// 2) Build the cube grid.
	cg := &CubeGrid{conn: conn}
	if err := cg.BuildGrid(3, 3, 2.0); err != nil {
		fmt.Printf("Error building grid: %v\n", err)
		return
	}

	// 3) Attach a pole of 5 cubes to the grid's center cube.
	if err := cg.AddPole(5, 2.0); err != nil {
		fmt.Printf("Error adding pole: %v\n", err)
		return
	}

	// Wait a bit to see the structure before dismantling.
	time.Sleep(5 * time.Second)

	// 4) Cleanup all cubes.
	cg.Dismantle()
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
