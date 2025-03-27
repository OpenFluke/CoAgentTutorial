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
	"sync"
	"time"
)

const (
	ServerIP      = "127.0.0.1"
	ServerPort    = 14000
	SharedPass    = "my_secure_password"
	EndOfMsg      = "<???DONE???---"
	SocketTimeout = 5 * time.Second
	ParamDelay    = 200 * time.Millisecond
)

// TCPClient wraps a net.Conn with a mutex for atomic writes.
type TCPClient struct {
	conn net.Conn
	mu   sync.Mutex
}

func createTCPClient() (*TCPClient, error) {
	conn, err := net.Dial("tcp", fmt.Sprintf("%s:%d", ServerIP, ServerPort))
	if err != nil {
		return nil, err
	}
	if err := sendMessage(conn, SharedPass); err != nil {
		conn.Close()
		return nil, err
	}
	authResp, err := receiveMessage(conn)
	if err != nil {
		conn.Close()
		return nil, err
	}
	if authResp != "auth_success" {
		conn.Close()
		return nil, fmt.Errorf("authentication failed: %s", authResp)
	}
	return &TCPClient{conn: conn}, nil
}

type CubeGrid struct {
	Rows       int
	Cols       int
	Spacing    float64
	BaseX      float64
	BaseY      float64
	BaseZ      float64
	CubeNames  []string
	JointNames []string
}

type CubePole struct {
	NumPole    int
	BasePos    []float64
	CubeNames  []string
	JointNames []string
}

func main() {
	// PHASE 1: Create TCP connections.
	gridRows, gridCols := 3, 3
	totalGrid := gridRows * gridCols
	gridClients := make([]*TCPClient, totalGrid)
	var wg sync.WaitGroup
	for i := 0; i < totalGrid; i++ {
		wg.Add(1)
		go func(i int) {
			defer wg.Done()
			client, err := createTCPClient()
			if err != nil {
				fmt.Printf("Grid client %d creation error: %v\n", i, err)
				return
			}
			gridClients[i] = client
		}(i)
	}
	wg.Wait()
	fmt.Printf("Created %d grid TCP connections.\n", totalGrid)

	numPole := 5 // Set to 9 cubes as requested
	poleClients := make([]*TCPClient, numPole)
	for i := 0; i < numPole; i++ {
		wg.Add(1)
		go func(i int) {
			defer wg.Done()
			client, err := createTCPClient()
			if err != nil {
				fmt.Printf("Pole client %d creation error: %v\n", i, err)
				return
			}
			poleClients[i] = client
		}(i)
	}
	wg.Wait()
	fmt.Printf("Created %d pole TCP connections.\n", numPole)

	// PHASE 2: Spawn grid cubes concurrently.
	initialGridList, err := getCubeList(gridClients[0].conn)
	if err != nil {
		fmt.Printf("Error getting initial grid cube list: %v\n", err)
		return
	}
	oldGridCubes := append([]string(nil), initialGridList.Cubes...)

	grid := CubeGrid{
		Rows:    gridRows,
		Cols:    gridCols,
		Spacing: 2.0,
		BaseX:   20.0,
		BaseY:   100.0,
		BaseZ:   0.0,
	}
	grid.CubeNames = make([]string, totalGrid)
	startX := grid.BaseX - float64(gridCols-1)*grid.Spacing/2.0
	startZ := grid.BaseZ - float64(gridRows-1)*grid.Spacing/2.0
	y := grid.BaseY

	wg = sync.WaitGroup{}
	for i := 0; i < totalGrid; i++ {
		wg.Add(1)
		go func(i int) {
			defer wg.Done()
			row := i / grid.Cols
			col := i % grid.Cols
			pos := []float64{
				startX + float64(col)*grid.Spacing,
				y,
				startZ + float64(row)*grid.Spacing,
			}
			client := gridClients[i]
			client.mu.Lock()
			cubeName, err := spawnCube(client.conn, pos, []float64{0, 0, 0}, true, totalGrid, i, oldGridCubes)
			client.mu.Unlock()
			if err != nil {
				fmt.Printf("Error spawning grid cube at (%d,%d): %v\n", row, col, err)
				return
			}
			grid.CubeNames[i] = cubeName
			fmt.Printf("Grid cube spawned: %s at %v\n", cubeName, pos)
		}(i)
	}
	wg.Wait()
	fmt.Println("All grid cubes spawned.")

	// PHASE 3: Connect grid cubes concurrently.
	var gridJointNames []string
	var jointNamesMu sync.Mutex // Protects gridJointNames
	var jointWG sync.WaitGroup
	// Horizontal joints.
	for row := 0; row < grid.Rows; row++ {
		for col := 0; col < grid.Cols-1; col++ {
			idxA := row*grid.Cols + col
			idxB := row*grid.Cols + (col + 1)
			posA := []float64{
				startX + float64(col)*grid.Spacing,
				y,
				startZ + float64(row)*grid.Spacing,
			}
			posB := []float64{
				startX + float64(col+1)*grid.Spacing,
				y,
				startZ + float64(row)*grid.Spacing,
			}
			anchor := midPoint(posA, posB)
			client := gridClients[idxA%len(gridClients)]
			jointWG.Add(1)
			go func(idxA, idxB int, anchor []float64, client *TCPClient) {
				defer jointWG.Done()
				client.mu.Lock()
				jn := createJointAt(client.conn, grid.CubeNames[idxA], grid.CubeNames[idxB], "hinge", anchor, nil)
				client.mu.Unlock()
				if jn != "" {
					fmt.Printf("Horizontal joint created: %s between %s and %s\n", jn, grid.CubeNames[idxA], grid.CubeNames[idxB])
					jointNamesMu.Lock()
					gridJointNames = append(gridJointNames, jn)
					jointNamesMu.Unlock()
				}
			}(idxA, idxB, anchor, client)
		}
	}
	// Vertical joints.
	for col := 0; col < grid.Cols; col++ {
		for row := 0; row < grid.Rows-1; row++ {
			idxA := row*grid.Cols + col
			idxB := (row+1)*grid.Cols + col
			posA := []float64{
				startX + float64(col)*grid.Spacing,
				y,
				startZ + float64(row)*grid.Spacing,
			}
			posB := []float64{
				startX + float64(col)*grid.Spacing,
				y,
				startZ + float64(row+1)*grid.Spacing,
			}
			anchor := midPoint(posA, posB)
			client := gridClients[idxA%len(gridClients)]
			jointWG.Add(1)
			go func(idxA, idxB int, anchor []float64, client *TCPClient) {
				defer jointWG.Done()
				client.mu.Lock()
				jn := createJointAt(client.conn, grid.CubeNames[idxA], grid.CubeNames[idxB], "hinge", anchor, nil)
				client.mu.Unlock()
				if jn != "" {
					fmt.Printf("Vertical joint created: %s between %s and %s\n", jn, grid.CubeNames[idxA], grid.CubeNames[idxB])
					jointNamesMu.Lock()
					gridJointNames = append(gridJointNames, jn)
					jointNamesMu.Unlock()
				}
			}(idxA, idxB, anchor, client)
		}
	}
	jointWG.Wait()
	fmt.Printf("All grid joints created (%d joints).\n", len(gridJointNames))

	// PHASE 4: Spawn and connect pole cubes.
	// PHASE 4: Spawn and connect pole cubes.
	centerIdx := (grid.Rows/2)*grid.Cols + grid.Cols/2
	centerCube := grid.CubeNames[centerIdx]
	centerPos, err := getCubePosition(gridClients[centerIdx].conn, centerCube)
	if err != nil || centerCube == "" {
		fmt.Printf("Error fetching center cube position: %v\n", err)
		return
	}

	pole := CubePole{
		NumPole: numPole,
		BasePos: centerPos,
	}
	pole.CubeNames = make([]string, numPole)
	initialPoleList, err := getCubeList(poleClients[0].conn)
	if err != nil {
		fmt.Printf("Error getting initial pole cube list: %v\n", err)
		return
	}
	// Use the list if needed, or remove if unused:
	// oldPoleCubes := append([]string(nil), initialPoleList.Cubes...)

	wg = sync.WaitGroup{}
	for i := 0; i < numPole; i++ {
		wg.Add(1)
		go func(i int) {
			defer wg.Done()
			newPos := []float64{
				centerPos[0],
				centerPos[1] + float64(i+1)*2.0, // using constant vertical spacing directly
				centerPos[2],
			}
			client := poleClients[i]
			client.mu.Lock()
			cubeName, err := spawnCube(client.conn, newPos, []float64{0, 0, 0}, false, numPole, i, initialPoleList.Cubes)
			if err != nil {
				fmt.Printf("Error spawning pole cube %d: %v\n", i, err)
				client.mu.Unlock()
				return
			}
			pole.CubeNames[i] = cubeName
			applyPhysics(client.conn, cubeName, map[string][]float64{
				"central_impulse": {0, 10.0, 0},
			})
			client.mu.Unlock()
			fmt.Printf("Pole cube spawned: %s at %v\n", cubeName, newPos)
		}(i)
	}
	wg.Wait()
	fmt.Println("All pole cubes spawned.")

	// Connect pole cubes with joints.
	var poleJointNames []string
	var poleJointNamesMu sync.Mutex // Protects poleJointNames
	jointWG = sync.WaitGroup{}
	for i := 0; i < numPole; i++ {
		var posA, posB []float64
		var cubeA, cubeB string
		var client *TCPClient
		if i == 0 {
			posA = centerPos
			cubeA = centerCube
			posB, err = getCubePosition(poleClients[0].conn, pole.CubeNames[0])
			if err != nil {
				fmt.Printf("Error getting position for pole cube 0: %v\n", err)
				continue
			}
			cubeB = pole.CubeNames[0]
			client = poleClients[0]
		} else {
			client = poleClients[i]
			posA, err = getCubePosition(client.conn, pole.CubeNames[i-1])
			if err != nil {
				fmt.Printf("Error getting position for pole cube %d: %v\n", i-1, err)
				continue
			}
			cubeA = pole.CubeNames[i-1]
			posB, err = getCubePosition(client.conn, pole.CubeNames[i])
			if err != nil {
				fmt.Printf("Error getting position for pole cube %d: %v\n", i, err)
				continue
			}
			cubeB = pole.CubeNames[i]
		}
		anchor := midPoint(posA, posB)
		jointWG.Add(1)
		go func(client *TCPClient, cubeA, cubeB string, anchor []float64) {
			defer jointWG.Done()
			params := map[string]float64{
				"limit_upper":           0.0,
				"limit_lower":           0.0,
				"motor_enable":          1.0,
				"motor_target_velocity": 0.0,
				"motor_max_impulse":     1000.0,
			}
			client.mu.Lock()
			jn := createJointAt(client.conn, cubeA, cubeB, "hinge", anchor, params)
			client.mu.Unlock()
			if jn != "" {
				fmt.Printf("Pole joint created: %s between %s and %s\n", jn, cubeA, cubeB)
				poleJointNamesMu.Lock()
				poleJointNames = append(poleJointNames, jn)
				poleJointNamesMu.Unlock()
			}
		}(client, cubeA, cubeB, anchor)
	}
	jointWG.Wait()
	fmt.Printf("All pole joints created (%d joints).\n", len(poleJointNames))

	//var poleJointNames []string
	jointWG = sync.WaitGroup{}
	for i := 0; i < numPole; i++ {
		var posA, posB []float64
		var cubeA, cubeB string
		var client *TCPClient
		if i == 0 {
			posA = centerPos
			cubeA = centerCube
			posB, err = getCubePosition(poleClients[0].conn, pole.CubeNames[0])
			if err != nil {
				fmt.Printf("Error getting position for pole cube 0: %v\n", err)
				continue
			}
			cubeB = pole.CubeNames[0]
			client = poleClients[0]
		} else {
			client = poleClients[i]
			posA, err = getCubePosition(client.conn, pole.CubeNames[i-1])
			if err != nil {
				fmt.Printf("Error getting position for pole cube %d: %v\n", i-1, err)
				continue
			}
			cubeA = pole.CubeNames[i-1]
			posB, err = getCubePosition(client.conn, pole.CubeNames[i])
			if err != nil {
				fmt.Printf("Error getting position for pole cube %d: %v\n", i, err)
				continue
			}
			cubeB = pole.CubeNames[i]
		}
		anchor := midPoint(posA, posB)
		jointWG.Add(1)
		go func(client *TCPClient, cubeA, cubeB string, anchor []float64) {
			defer jointWG.Done()
			params := map[string]float64{
				"limit_upper":           0.0,
				"limit_lower":           0.0,
				"motor_enable":          1.0,
				"motor_target_velocity": 0.0,
				"motor_max_impulse":     1000.0,
			}
			client.mu.Lock()
			jn := createJointAt(client.conn, cubeA, cubeB, "hinge", anchor, params)
			client.mu.Unlock()
			if jn != "" {
				fmt.Printf("Pole joint created: %s between %s and %s\n", jn, cubeA, cubeB)
				poleJointNames = appendSafeString(&poleJointNames, jn)
			}
		}(client, cubeA, cubeB, anchor)
	}
	jointWG.Wait()
	fmt.Printf("All pole joints created (%d joints).\n", len(poleJointNames))

	// PHASE 5: Rotate the pivot joint of the pole (only the upper half bends)
	// For a pole of 5 cubes, use the joint connecting cube 1 and cube 2 as the pivot.
	pivotJointIndex := 1 // if pole cubes are indexed 0-4, pivot between cube 1 and cube 2

	if len(poleJointNames) > pivotJointIndex {
		client := poleClients[pivotJointIndex+1]     // client controlling the cube above the pivot
		jointName := poleJointNames[pivotJointIndex] // pivot joint

		// Fold the arm 90 degrees (only the top half rotates)
		fmt.Println("Folding the upper half of the pole 90 degrees...")
		setJointParam(client.conn, jointName, "motor_target_velocity", 1.0)
		time.Sleep(1570 * time.Millisecond) // ~90 degrees (π/2 radians)
		setJointParam(client.conn, jointName, "motor_target_velocity", 0.0)

		// Pause to observe folded state
		fmt.Println("Upper half folded. Pausing for 5 seconds...")
		time.Sleep(5 * time.Second)

		// Fold back to original position
		fmt.Println("Folding back to original position...")
		setJointParam(client.conn, jointName, "motor_target_velocity", -1.0)
		time.Sleep(1570 * time.Millisecond)
		setJointParam(client.conn, jointName, "motor_target_velocity", 0.0)

		fmt.Println("Rotation complete.")
	} else {
		fmt.Println("Not enough joints to rotate the designated pivot.")
	}

	time.Sleep(5000 * time.Millisecond)

	// PHASE 6: Cleanup.
	fmt.Println("Dismantling all cubes...")
	wg = sync.WaitGroup{}
	for i := 0; i < totalGrid; i++ {
		wg.Add(1)
		go func(i int) {
			defer wg.Done()
			client := gridClients[i]
			client.mu.Lock()
			despawnCube(client.conn, grid.CubeNames[i])
			client.mu.Unlock()
		}(i)
	}
	for i := 0; i < numPole; i++ {
		wg.Add(1)
		go func(i int) {
			defer wg.Done()
			client := poleClients[i]
			client.mu.Lock()
			despawnCube(client.conn, pole.CubeNames[i])
			client.mu.Unlock()
		}(i)
	}
	wg.Wait()
	fmt.Println("Cleanup complete. Exiting.")

	for _, c := range gridClients {
		if c != nil {
			c.conn.Close()
		}
	}
	for _, c := range poleClients {
		if c != nil {
			c.conn.Close()
		}
	}

}

// Helper Functions (unchanged from original)
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

func spawnCube(conn net.Conn, position, rotation []float64, isBase bool, expectedTotal, myIndex int, oldCubes []string) (string, error) {
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
		if len(newCubes) >= expectedTotal {
			sort.Strings(newCubes)
			if myIndex < len(newCubes) {
				newCubeName = newCubes[myIndex]
				break
			}
		}
	}
	if newCubeName == "" {
		return "", errors.New("spawn_cube timed out: no new cube found")
	}
	fmt.Printf("[spawnCube] Created: %s at %v\n", newCubeName, position)
	return newCubeName, nil
}

func createJointAt(conn net.Conn, cubeA, cubeB, jointType string, anchor []float64, params map[string]float64) string {
	const maxRetries = 3
	var jointName string
	cmd := map[string]interface{}{
		"type":       "create_joint",
		"cube1":      cubeA,
		"cube2":      cubeB,
		"joint_type": jointType,
		"position":   anchor,
	}
	if params != nil {
		cmd["joint_params"] = params
	}
	out, _ := json.Marshal(cmd)
	for attempt := 0; attempt < maxRetries; attempt++ {
		if err := sendMessage(conn, string(out)); err != nil {
			fmt.Printf("create_joint send error: %v\n", err)
			time.Sleep(ParamDelay)
			continue
		}
		resp, err := receiveMessage(conn)
		if err != nil {
			fmt.Printf("create_joint read error: %v\n", err)
			time.Sleep(ParamDelay)
			continue
		}
		var jr struct {
			Type      string `json:"type"`
			JointName string `json:"joint_name"`
		}
		if err := json.Unmarshal([]byte(resp), &jr); err != nil {
			fmt.Printf("createJointAt parse error: %v\n", err)
			time.Sleep(ParamDelay)
			continue
		}
		if jr.Type != "joint_created" {
			fmt.Printf("Unexpected create_joint response: %s\n", resp)
			time.Sleep(ParamDelay)
			continue
		}
		jointName = jr.JointName
		break
	}
	fmt.Printf("[createJointAt] Created '%s' between %s and %s\n", jointName, cubeA, cubeB)
	return jointName
}

func setJointParam(conn net.Conn, jointName, paramName string, value float64) {
	const maxRetries = 10
	for attempt := 0; attempt < maxRetries; attempt++ {
		cmd := map[string]interface{}{
			"type":       "set_joint_param",
			"joint_name": jointName,
			"param_name": paramName,
			"value":      value,
		}
		out, _ := json.Marshal(cmd)
		if err := sendMessage(conn, string(out)); err != nil {
			fmt.Printf("setJointParam send error: %v\n", err)
			time.Sleep(ParamDelay)
			continue
		}
		resp, err := receiveMessage(conn)
		if err != nil {
			fmt.Printf("setJointParam receive error: %v\n", err)
			time.Sleep(ParamDelay)
			continue
		}
		if strings.Contains(resp, "not found") {
			fmt.Printf("Joint %s not found (attempt %d), retrying...\n", jointName, attempt+1)
			time.Sleep(ParamDelay)
			continue
		}
		break
	}
}

func despawnCube(conn net.Conn, cubeName string) {
	cmd := map[string]interface{}{
		"type":      "despawn_cube",
		"cube_name": cubeName,
	}
	out, _ := json.Marshal(cmd)
	_ = sendMessage(conn, string(out))
	_, _ = receiveMessage(conn)
}

func applyPhysics(conn net.Conn, cubeName string, physics map[string][]float64) {
	cmd := map[string]interface{}{
		"type": "apply_physics",
	}
	for key, value := range physics {
		cmd[key] = value
	}
	out, _ := json.Marshal(cmd)
	if err := sendMessage(conn, string(out)); err != nil {
		fmt.Printf("applyPhysics error for %s: %v\n", cubeName, err)
	}
}

func getCubeList(conn net.Conn) (struct {
	Type  string   `json:"type"`
	Cubes []string `json:"cubes"`
}, error) {
	var result struct {
		Type  string   `json:"type"`
		Cubes []string `json:"cubes"`
	}
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

func midPoint(a, b []float64) []float64 {
	return []float64{
		(a[0] + b[0]) * 0.5,
		(a[1] + b[1]) * 0.5,
		(a[2] + b[2]) * 0.5,
	}
}

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

func appendSafeString(slice *[]string, s string) []string {
	*slice = append(*slice, s)
	return *slice
}
