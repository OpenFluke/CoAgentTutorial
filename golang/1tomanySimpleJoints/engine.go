package main

import (
	"bytes"
	"encoding/json"
	"fmt"
	"io"
	"net"
	"strings"
	"time"
)

const (
	SERVER_IP      = "192.168.0.116" // Replace with your server IP
	SERVER_PORT    = 14000
	SHARED_PASS    = "my_secure_password"
	END_OF_MSG     = "<???DONE???---"
	SOCKET_TIMEOUT = 2 * time.Second
	NUM_CUBES      = 10              // Number of cubes in the snake
	TEST_DURATION  = 5 * time.Second // Duration to observe each snake
)

// Supported joint types.
var jointTypes = []string{"pin", "hinge", "slider", "conetwist"}

// CubeListResponse struct to parse the server's cube list response.
type CubeListResponse struct {
	Type  string   `json:"type"`
	Cubes []string `json:"cubes"`
}

// sendMessage appends the end-of-message marker and writes to the connection.
func sendMessage(conn net.Conn, message string) error {
	fullMsg := message + END_OF_MSG
	_, err := conn.Write([]byte(fullMsg))
	if err != nil {
		return fmt.Errorf("failed to send message: %v", err)
	}
	return nil
}

// receiveMessage reads from the connection until it finds the END_OF_MSG marker.
func receiveMessage(conn net.Conn) (string, error) {
	conn.SetReadDeadline(time.Now().Add(SOCKET_TIMEOUT))
	var buffer bytes.Buffer
	tmp := make([]byte, 1024)
	for {
		n, err := conn.Read(tmp)
		if err != nil {
			if err == io.EOF {
				break
			}
			// If we timed out but have some data, break out.
			if netErr, ok := err.(net.Error); ok && netErr.Timeout() && buffer.Len() > 0 {
				break
			}
			return "", fmt.Errorf("read error: %v", err)
		}
		buffer.Write(tmp[:n])
		if bytes.Contains(buffer.Bytes(), []byte(END_OF_MSG)) {
			break
		}
	}
	result := buffer.String()
	idx := strings.Index(result, END_OF_MSG)
	if idx >= 0 {
		result = result[:idx]
	}
	return strings.TrimSpace(result), nil
}

func main() {
	fmt.Println("Starting program to test snake structures with different joint types...")

	// Connect to the server.
	conn, err := net.Dial("tcp", fmt.Sprintf("%s:%d", SERVER_IP, SERVER_PORT))
	if err != nil {
		fmt.Printf("Failed to connect to server: %v\n", err)
		return
	}
	defer conn.Close()

	// Authenticate with the server.
	if err := sendMessage(conn, SHARED_PASS); err != nil {
		fmt.Printf("Authentication send error: %v\n", err)
		return
	}
	authResp, err := receiveMessage(conn)
	if err != nil || authResp != "auth_success" {
		fmt.Printf("Authentication failed: %s %v\n", authResp, err)
		return
	}
	fmt.Println("Authenticated successfully")

	// Test each joint type.
	for _, jointType := range jointTypes {
		fmt.Printf("\n=== Testing joint type: %s ===\n", jointType)

		// Step 1: Spawn cubes for the snake.
		var cubeNames []string
		for i := 0; i < NUM_CUBES; i++ {
			spawnPos := []float64{float64(i * 2), 100, 40} // Space cubes 2 units apart along the X-axis.
			spawnCubeCmd := map[string]interface{}{
				"type":     "spawn_cube",
				"position": spawnPos,
				"rotation": []float64{0, 0, 0},
			}
			cmdBytes, _ := json.Marshal(spawnCubeCmd)
			if err := sendMessage(conn, string(cmdBytes)); err != nil {
				fmt.Printf("Failed to spawn cube %d: %v\n", i, err)
				return
			}
			fmt.Printf("Spawned cube %d at %v\n", i, spawnPos)
			time.Sleep(100 * time.Millisecond) // Small delay to ensure the server processes each command.
		}

		// Wait briefly for all cubes to spawn.
		time.Sleep(1 * time.Second)

		// Step 2: Get the list of spawned cubes.
		getCubeListCmd := map[string]interface{}{
			"type": "get_cube_list",
		}
		cmdBytes, _ := json.Marshal(getCubeListCmd)
		if err := sendMessage(conn, string(cmdBytes)); err != nil {
			fmt.Printf("Failed to send get_cube_list: %v\n", err)
			return
		}

		resp, err := receiveMessage(conn)
		if err != nil {
			fmt.Printf("Failed to receive cube list: %v\n", err)
			return
		}
		var cubeListResp CubeListResponse
		if err := json.Unmarshal([]byte(resp), &cubeListResp); err != nil {
			fmt.Printf("Failed to parse cube list response: %v\n", err)
			return
		}
		if cubeListResp.Type != "cube_list" {
			fmt.Printf("Invalid response type: %s\n", cubeListResp.Type)
			return
		}
		// Take the last NUM_CUBES from the list (assumes the server appends new cubes).
		allCubeNames := cubeListResp.Cubes
		cubeNames = allCubeNames[len(allCubeNames)-NUM_CUBES:]
		fmt.Printf("Cube names for %s snake: %v\n", jointType, cubeNames)

		// Step 3: Connect cubes with the current joint type.
		for i := 0; i < len(cubeNames)-1; i++ {
			createJointCmd := map[string]interface{}{
				"type":       "create_joint",
				"cube1":      cubeNames[i],
				"cube2":      cubeNames[i+1],
				"joint_type": jointType,
			}
			jointCmdBytes, _ := json.Marshal(createJointCmd)
			if err := sendMessage(conn, string(jointCmdBytes)); err != nil {
				fmt.Printf("Failed to create %s joint between %s and %s: %v\n", jointType, cubeNames[i], cubeNames[i+1], err)
				continue
			}
			// Wait for and verify the joint creation response.
			jointRespStr, err := receiveMessage(conn)
			if err != nil {
				fmt.Printf("Failed to receive joint creation response: %v\n", err)
				continue
			}
			var jointResp map[string]interface{}
			if err := json.Unmarshal([]byte(jointRespStr), &jointResp); err != nil {
				fmt.Printf("Failed to parse joint creation response: %v\n", err)
				continue
			}
			respType, ok := jointResp["type"].(string)
			if !ok || respType != "joint_created" {
				fmt.Printf("Invalid joint creation response type: %v\n", jointResp["type"])
				continue
			}
			fmt.Printf("Created %s joint between %s and %s\n", jointType, cubeNames[i], cubeNames[i+1])
			time.Sleep(100 * time.Millisecond)
		}

		// Step 4: Move the head of the snake.
		headCube := cubeNames[0]
		for i := 0; i < 100; i++ { // Apply force 10 times for visible movement.
			applyForceCmd := map[string]interface{}{
				"type":      "apply_force",
				"cube_name": headCube,
				"force":     []float64{10, 0, 0}, // Small force along X-axis.
			}
			forceCmdBytes, _ := json.Marshal(applyForceCmd)
			if err := sendMessage(conn, string(forceCmdBytes)); err != nil {
				fmt.Printf("Failed to apply force to %s: %v\n", headCube, err)
			} else {
				fmt.Printf("Applied force to head cube %s\n", headCube)
			}
			time.Sleep(100 * time.Millisecond)
		}

		// Step 5: Observe the snake's behavior for 5 seconds.
		fmt.Println("Observing snake behavior for 5 seconds...")
		time.Sleep(TEST_DURATION)

		// Step 6: Despawn the cubes.
		for _, cubeName := range cubeNames {
			despawnCubeCmd := map[string]interface{}{
				"type":      "despawn_cube",
				"cube_name": cubeName,
			}
			despawnCmdBytes, _ := json.Marshal(despawnCubeCmd)
			if err := sendMessage(conn, string(despawnCmdBytes)); err != nil {
				fmt.Printf("Failed to despawn cube %s: %v\n", cubeName, err)
			} else {
				fmt.Printf("Despawned cube %s\n", cubeName)
			}
			time.Sleep(100 * time.Millisecond)
		}
		fmt.Printf("=== Finished testing %s joint type ===\n", jointType)
	}

	fmt.Println("All joint types tested successfully!")
}
