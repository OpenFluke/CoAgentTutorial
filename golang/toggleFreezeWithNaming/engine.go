package main

import (
	"bufio"
	"encoding/json"
	"fmt"
	"net"
	"strings"
	"time"
)

const (
	serverAddr = "127.0.0.1:14000"    // Your CubeSyncServer IP:PORT
	authPass   = "my_secure_password" // Replace with your shared secret
	delimiter  = "<???DONE???---"
)

type Message map[string]interface{}

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

// spawnCubeWithName creates its own TCP connection, authenticates, and spawns a cube
// using the provided cubeName. It then toggles unfreeze and freeze commands.
// The server will append "_BASE" to the cube name since is_base is true.
func spawnCubeWithName(cubeName string) {
	conn, err := net.Dial("tcp", serverAddr)
	if err != nil {
		fmt.Println("[Preload] Failed to connect:", err)
		return
	}
	defer conn.Close()

	// Authenticate
	if _, err := conn.Write([]byte(authPass + delimiter)); err != nil {
		fmt.Println("[Preload] Auth write error:", err)
		return
	}
	authResp, err := readResponse(conn)
	if err != nil {
		fmt.Println("[Preload] Failed to read auth response:", err)
		return
	}
	fmt.Println("[Preload] Auth response:", authResp)

	// Spawn cube with the provided cube_name
	spawn := Message{
		"type":      "spawn_cube",
		"cube_name": cubeName,
		"position":  []float64{0, 120, 0},
		"rotation":  []float64{0, 0, 0},
		"is_base":   true,
	}
	if err := sendJSONMessage(conn, spawn); err != nil {
		fmt.Println("[Preload] Failed to spawn cube:", err)
		return
	}
	fmt.Println("[Preload] Spawn message sent for cube", cubeName)

	// Since we're specifying a name, the server will append "_BASE"
	fullCubeName := cubeName + "_BASE"
	fmt.Println("[Preload] Expecting cube name:", fullCubeName)

	// Wait briefly for the cube to be created on the server
	time.Sleep(1 * time.Second)

	// Unfreeze the cube so it can fall (enable physics)
	unfreeze := Message{
		"type":      "freeze_cube",
		"cube_name": fullCubeName,
		"freeze":    false,
	}
	if err := sendJSONMessage(conn, unfreeze); err != nil {
		fmt.Println("[Preload] Failed to unfreeze cube:", err)
		return
	}
	fmt.Println("[Preload] Cube unfreeze command sent for", fullCubeName)
	time.Sleep(1 * time.Second)

	// Freeze the cube again
	freeze := Message{
		"type":      "freeze_cube",
		"cube_name": fullCubeName,
		"freeze":    true,
	}
	if err := sendJSONMessage(conn, freeze); err != nil {
		fmt.Println("[Preload] Failed to freeze cube:", err)
		return
	}
	fmt.Println("[Preload] Cube freeze command sent for", fullCubeName)
	time.Sleep(2 * time.Second)

	// Optionally, despawn the cube
	despawn := Message{
		"type":      "despawn_cube",
		"cube_name": fullCubeName,
	}
	if err := sendJSONMessage(conn, despawn); err != nil {
		fmt.Println("[Preload] Failed to despawn cube:", err)
		return
	}
	fmt.Println("[Preload] Cube despawned:", fullCubeName)
}

func main() {
	spawnCubeWithName("TestCube")
}
