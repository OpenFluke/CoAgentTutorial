package main

import (
	"bufio"
	"encoding/json"
	"fmt"
	"net"
	"sort"
	"strings"
	"time"
)

const (
	serverAddr = "127.0.0.1:14000"    // Your CubeSyncServer IP:PORT
	authPass   = "my_secure_password" // Replace with your actual shared password
	delimiter  = "<???DONE???---"
)

type Message map[string]interface{}

// CubeListResponse represents the expected response format from "get_cube_list"
type CubeListResponse struct {
	Type  string   `json:"type"`
	Cubes []string `json:"cubes"`
}

// sendJSONMessage marshals and sends a JSON message with the delimiter appended.
func sendJSONMessage(conn net.Conn, msg Message) error {
	data, err := json.Marshal(msg)
	if err != nil {
		return err
	}
	data = append(data, []byte(delimiter)...)
	_, err = conn.Write(data)
	return err
}

// readResponse reads from the connection until the delimiter is found.
func readResponse(conn net.Conn) (string, error) {
	reader := bufio.NewReader(conn)
	conn.SetReadDeadline(time.Now().Add(3 * time.Second))
	var builder strings.Builder
	for {
		line, err := reader.ReadString('-')
		if err != nil {
			break
		}
		builder.WriteString(line)
		if strings.Contains(line, delimiter) {
			break
		}
	}
	full := strings.ReplaceAll(builder.String(), delimiter, "")
	return strings.TrimSpace(full), nil
}

// getCubeList sends the "get_cube_list" command and parses the response.
func getCubeList(conn net.Conn) ([]string, error) {
	req := Message{"type": "get_cube_list"}
	if err := sendJSONMessage(conn, req); err != nil {
		return nil, err
	}
	respStr, err := readResponse(conn)
	if err != nil {
		return nil, err
	}
	var cubeListResp CubeListResponse
	if err := json.Unmarshal([]byte(respStr), &cubeListResp); err != nil {
		return nil, err
	}
	return cubeListResp.Cubes, nil
}

// difference returns the elements in newList that are not in baseList.
func difference(baseList, newList []string) []string {
	baseMap := make(map[string]bool)
	for _, name := range baseList {
		baseMap[name] = true
	}
	var diff []string
	for _, name := range newList {
		if !baseMap[name] {
			diff = append(diff, name)
		}
	}
	sort.Strings(diff)
	return diff
}

func main() {
	conn, err := net.Dial("tcp", serverAddr)
	if err != nil {
		panic("Failed to connect: " + err.Error())
	}
	defer conn.Close()

	// Step 1: Authenticate
	conn.Write([]byte(authPass + delimiter))
	authResp, err := readResponse(conn)
	if err != nil {
		fmt.Println("Failed to read auth response:", err)
		return
	}
	fmt.Println("Auth response:", authResp)

	// Get baseline cube list
	baseCubes, err := getCubeList(conn)
	if err != nil {
		fmt.Println("Failed to get baseline cube list:", err)
		return
	}
	fmt.Println("Baseline cubes:", baseCubes)

	// Step 2: Spawn cube (no cube_name provided)
	spawn := Message{
		"type":     "spawn_cube",
		"position": []float64{0, 120, 0},
		"rotation": []float64{0, 0, 0},
		"is_base":  true,
	}
	if err := sendJSONMessage(conn, spawn); err != nil {
		fmt.Println("Failed to spawn cube:", err)
		return
	}
	fmt.Println("Spawn message sent. Waiting for cube to be created...")

	// Wait a bit longer (e.g., 2 seconds) to let the server update its cube list
	time.Sleep(1 * time.Second)

	// Step 3: Get new cube list
	newCubes, err := getCubeList(conn)
	if err != nil {
		fmt.Println("Failed to get new cube list:", err)
		return
	}
	fmt.Println("New cube list:", newCubes)

	// Determine the newly spawned cube by computing the difference.
	diff := difference(baseCubes, newCubes)
	if len(diff) == 0 {
		fmt.Println("Could not determine new cube name from difference.")
		fmt.Printf("Baseline: %v, New: %v\n", baseCubes, newCubes)
		return
	}
	cubeName := diff[0]
	fmt.Println("Detected new cube name:", cubeName)

	// Step 4: Unfreeze the cube
	unfreeze := Message{
		"type":      "freeze_cube",
		"cube_name": cubeName,
		"freeze":    false,
	}
	if err := sendJSONMessage(conn, unfreeze); err != nil {
		fmt.Println("Failed to unfreeze cube:", err)
		return
	}
	fmt.Println("Cube unfrozen")
	time.Sleep(1 * time.Second)

	// Step 5: Freeze the cube again
	freeze := Message{
		"type":      "freeze_cube",
		"cube_name": cubeName,
		"freeze":    true,
	}
	if err := sendJSONMessage(conn, freeze); err != nil {
		fmt.Println("Failed to freeze cube:", err)
		return
	}
	fmt.Println("Cube frozen again")
	time.Sleep(2 * time.Second)

	// Step 6: Despawn the cube
	despawn := Message{
		"type":      "despawn_cube",
		"cube_name": cubeName,
	}
	if err := sendJSONMessage(conn, despawn); err != nil {
		fmt.Println("Failed to despawn cube:", err)
		return
	}
	fmt.Println("Cube despawned")
}
