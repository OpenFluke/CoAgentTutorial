package main

import (
	"bytes"
	"encoding/json"
	"fmt"
	"io"
	"math"
	"net"
	"sync"
	"time"
)

// Configuration Constants
const (
	SERVER_IP      = "192.168.0.116"      // Server IP address
	SERVER_PORT    = 14000                // Server port
	SHARED_PASS    = "my_secure_password" // Authentication password
	END_OF_MSG     = "<???DONE???---"     // Message delimiter
	SOCKET_TIMEOUT = 1 * time.Second      // Timeout for receiving messages
	NUM_CUBES      = 10                   // Number of cubes to control
)

// Utility Functions

// sendMessage sends a message over the TCP connection with a delimiter
func sendMessage(conn net.Conn, message string) error {
	fullMsg := message + END_OF_MSG
	_, err := conn.Write([]byte(fullMsg))
	return err
}

// receiveMessage reads a message from the TCP connection until the delimiter or timeout
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
			if netErr, ok := err.(net.Error); ok && netErr.Timeout() && buffer.Len() > 0 {
				break
			}
			return buffer.String(), err
		}
		buffer.Write(tmp[:n])
		if bytes.Contains(buffer.Bytes(), []byte(END_OF_MSG)) {
			break
		}
	}
	result := buffer.String()
	idx := bytes.Index([]byte(result), []byte(END_OF_MSG))
	if idx >= 0 {
		result = result[:idx]
	}
	return result, nil
}

// controlCube manages a single cube over its own TCP connection
func controlCube(cubeID int) {
	// Establish TCP connection
	conn, err := net.Dial("tcp", fmt.Sprintf("%s:%d", SERVER_IP, SERVER_PORT))
	if err != nil {
		fmt.Printf("[Cube %d] Failed to connect: %v\n", cubeID, err)
		return
	}
	defer conn.Close()

	// Authenticate with the server
	if err := sendMessage(conn, SHARED_PASS); err != nil {
		fmt.Printf("[Cube %d] Authentication send error: %v\n", cubeID, err)
		return
	}
	msg, err := receiveMessage(conn)
	if err != nil || msg != "auth_success" {
		fmt.Printf("[Cube %d] Authentication failed: %s %v\n", cubeID, msg, err)
		return
	}

	// Spawn the cube at a unique position
	spawnPos := []float64{float64(cubeID * 10), 100, 40}
	spawnCubeCmd := map[string]interface{}{
		"type":     "spawn_cube",
		"position": spawnPos,
		"rotation": []float64{0, 0, 0},
	}
	spawnBytes, _ := json.Marshal(spawnCubeCmd)
	if err := sendMessage(conn, string(spawnBytes)); err != nil {
		fmt.Printf("[Cube %d] Failed to spawn cube: %v\n", cubeID, err)
		return
	}
	fmt.Printf("[Cube %d] Spawned at %v\n", cubeID, spawnPos)

	// Define target position (unique x based on cubeID)
	target := []float64{float64(cubeID * 10), 72.562, 30.294}

	// Control parameters
	const (
		maxIterations = 1000 // Maximum control iterations
		tolerance     = 5.0  // Distance tolerance to target
		k             = 0.05 // Proportional gain
		maxForce      = 10.0 // Maximum force magnitude
	)

	// Control loop
	for i := 0; i < maxIterations; i++ {
		// Request cube state
		getStateCmd := map[string]interface{}{"type": "get_cube_state"}
		stBytes, _ := json.Marshal(getStateCmd)
		if err := sendMessage(conn, string(stBytes)); err != nil {
			continue
		}

		resp, err := receiveMessage(conn)
		if err != nil || len(resp) == 0 {
			time.Sleep(100 * time.Millisecond)
			continue
		}

		// Parse cube state
		var st map[string]interface{}
		if err := json.Unmarshal([]byte(resp), &st); err != nil {
			fmt.Printf("[Cube %d] JSON decode error: %v\n", cubeID, err)
			continue
		}
		posIF, ok := st["position"].([]interface{})
		if !ok || len(posIF) != 3 {
			continue
		}
		pos := []float64{0, 0, 0}
		for j := 0; j < 3; j++ {
			pos[j], _ = posIF[j].(float64)
		}

		// Compute error vector and distance to target
		errVec := []float64{target[0] - pos[0], target[1] - pos[1], target[2] - pos[2]}
		dist := math.Sqrt(errVec[0]*errVec[0] + errVec[1]*errVec[1] + errVec[2]*errVec[2])
		if dist < tolerance {
			fmt.Printf("[Cube %d] Reached target at %v\n", cubeID, pos)
			break
		}

		// Compute force using proportional control
		force := []float64{k * errVec[0], k * errVec[1], k * errVec[2]}
		mag := math.Sqrt(force[0]*force[0] + force[1]*force[1] + force[2]*force[2])
		if mag > maxForce {
			scale := maxForce / mag
			force[0] *= scale
			force[1] *= scale
			force[2] *= scale
		}

		// Apply force to the cube
		applyForceCmd := map[string]interface{}{
			"type":  "apply_force",
			"force": force,
		}
		forceBytes, _ := json.Marshal(applyForceCmd)
		if err := sendMessage(conn, string(forceBytes)); err != nil {
			continue
		}

		// Log progress (optional, reduce frequency if too verbose)
		fmt.Printf("[Cube %d] Iteration %d: Pos: %.2f,%.2f,%.2f Force: %.2f,%.2f,%.2f\n",
			cubeID, i, pos[0], pos[1], pos[2], force[0], force[1], force[2])
		time.Sleep(100 * time.Millisecond) // Simulate real-time control
	}

	fmt.Printf("[Cube %d] Control finished.\n", cubeID)
}

// Main Function
func main() {
	fmt.Println("Starting AI model to control 10 cubes...")

	// Use WaitGroup to synchronize goroutines
	var wg sync.WaitGroup

	// Spawn 10 goroutines, one for each cube
	for cubeID := 0; cubeID < NUM_CUBES; cubeID++ {
		wg.Add(1)
		go func(id int) {
			defer wg.Done()
			controlCube(id)
		}(cubeID)
	}

	// Wait for all cubes to complete
	wg.Wait()
	fmt.Println("All cubes have been controlled successfully.")
}
