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
	SERVER_IP      = "127.0.0.1" // or "192.168.0.116" if needed
	SERVER_PORT    = 14000
	SHARED_PASS    = "my_secure_password"
	END_OF_MSG     = "<???DONE???---"
	SOCKET_TIMEOUT = 2 * time.Second
	NUM_CUBES      = 10              // Number of cubes in the snake
	TEST_DURATION  = 5 * time.Second // Duration to observe each snake
)

// We test these 5 types: 4 normal joints plus "bone"
var testTypes = []string{"pin", "hinge", "slider", "conetwist", "bone"}

// CubeListResponse struct to parse the server's "get_cube_list" response.
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
			// If we timed out but have some data, we break out to parse what we have
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
	fmt.Println("Starting program to test snake structures with various joint/bone types...")

	// 1) Connect to the server
	conn, err := net.Dial("tcp", fmt.Sprintf("%s:%d", SERVER_IP, SERVER_PORT))
	if err != nil {
		fmt.Printf("Failed to connect to server: %v\n", err)
		return
	}
	defer conn.Close()

	// 2) Authenticate with the server
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

	// 3) Loop over each test type (pin, hinge, slider, conetwist, bone)
	for _, t := range testTypes {
		fmt.Printf("\n=== Testing type: %s ===\n", t)

		// Step A: Spawn cubes
		var cubeNames []string
		for i := 0; i < NUM_CUBES; i++ {
			spawnPos := []float64{float64(i * 2), 100, 40} // space them along X
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
			time.Sleep(100 * time.Millisecond)
		}

		// Wait a bit for the server to finish spawning
		time.Sleep(1 * time.Second)

		// Step B: Get the list of spawned cubes
		getCubeListCmd := map[string]interface{}{
			"type": "get_cube_list",
		}
		if data, err := json.Marshal(getCubeListCmd); err == nil {
			if err := sendMessage(conn, string(data)); err != nil {
				fmt.Printf("Failed to send get_cube_list: %v\n", err)
				return
			}
		} else {
			fmt.Printf("Failed to marshal get_cube_list: %v\n", err)
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
		// Take the last NUM_CUBES from the list
		allCubeNames := cubeListResp.Cubes
		if len(allCubeNames) < NUM_CUBES {
			fmt.Println("Not enough cubes returned from server!")
			return
		}
		cubeNames = allCubeNames[len(allCubeNames)-NUM_CUBES:]
		fmt.Printf("Cube names for %s test: %v\n", t, cubeNames)

		// Step C: Connect consecutive cubes with either create_joint or create_bone
		for i := 0; i < len(cubeNames)-1; i++ {
			cubeA := cubeNames[i]
			cubeB := cubeNames[i+1]

			var createCmd map[string]interface{}
			if t == "bone" {
				// Use "create_bone" command
				createCmd = map[string]interface{}{
					"type":     "create_bone",
					"cube1":    cubeA,
					"cube2":    cubeB,
					"snake_id": "testSnake", // optional ID
				}
			} else {
				// Use "create_joint" command
				createCmd = map[string]interface{}{
					"type":       "create_joint",
					"cube1":      cubeA,
					"cube2":      cubeB,
					"joint_type": t,
				}
			}

			cmdBytes, _ := json.Marshal(createCmd)
			if err := sendMessage(conn, string(cmdBytes)); err != nil {
				fmt.Printf("Failed to create %s link between %s and %s: %v\n", t, cubeA, cubeB, err)
				continue
			}
			// Wait for and verify the response
			jointRespStr, err := receiveMessage(conn)
			if err != nil {
				fmt.Printf("Failed to receive link creation response: %v\n", err)
				continue
			}
			var jointResp map[string]interface{}
			if err := json.Unmarshal([]byte(jointRespStr), &jointResp); err != nil {
				fmt.Printf("Failed to parse link creation response: %v\n", err)
				continue
			}
			respType, ok := jointResp["type"].(string)
			if t == "bone" {
				if !ok || respType != "bone_created" {
					fmt.Printf("Invalid bone creation response: %v\n", jointResp["type"])
					continue
				}
				fmt.Printf("Created bone between %s and %s\n", cubeA, cubeB)
			} else {
				if !ok || respType != "joint_created" {
					fmt.Printf("Invalid joint creation response: %v\n", jointResp["type"])
					continue
				}
				fmt.Printf("Created %s joint between %s and %s\n", t, cubeA, cubeB)
			}
			time.Sleep(100 * time.Millisecond)
		}

		// Step D: Apply force to the "head" of the snake
		headCube := cubeNames[0]
		for i := 0; i < 10; i++ {
			applyForceCmd := map[string]interface{}{
				"type":      "apply_force",
				"cube_name": headCube,
				"force":     []float64{10, 0, 0}, // Force along +X
			}
			forceCmdBytes, _ := json.Marshal(applyForceCmd)
			if err := sendMessage(conn, string(forceCmdBytes)); err != nil {
				fmt.Printf("Failed to apply force to %s: %v\n", headCube, err)
			} else {
				fmt.Printf("Applied force to head cube %s\n", headCube)
			}
			time.Sleep(100 * time.Millisecond)
		}

		// Step E: Observe for 5 seconds
		fmt.Printf("Observing %s snake behavior for %v...\n", t, TEST_DURATION)
		time.Sleep(TEST_DURATION)

		// Step F: Despawn the cubes
		for _, c := range cubeNames {
			despawnCubeCmd := map[string]interface{}{
				"type":      "despawn_cube",
				"cube_name": c,
			}
			cmdBytes, _ := json.Marshal(despawnCubeCmd)
			if err := sendMessage(conn, string(cmdBytes)); err != nil {
				fmt.Printf("Failed to despawn cube %s: %v\n", c, err)
			} else {
				fmt.Printf("Despawned cube %s\n", c)
			}
			time.Sleep(100 * time.Millisecond)
		}

		fmt.Printf("=== Finished testing %s ===\n", t)
	}

	fmt.Println("All joint/bone tests completed successfully!")
}
