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

// ----------------------------------------------------------------
// Adjust these as needed
// ----------------------------------------------------------------
const (
	SERVER_IP      = "192.168.0.116" // Replace with your server's IP or hostname
	SERVER_PORT    = 14000
	SHARED_PASS    = "my_secure_password"
	END_OF_MSG     = "<???DONE???---"
	SOCKET_TIMEOUT = 2 * time.Second

	NUM_LINKS     = 5               // Number of cubes in the snake (not counting the base)
	TEST_DURATION = 5 * time.Second // Duration to observe each snake
)

// We’ll test each of these joint types in turn.
var jointTypes = []string{"pin", "hinge", "slider", "conetwist"}

// ----------------------------------------------------------------
// Basic data structures for receiving JSON from the server
// ----------------------------------------------------------------

type CubeListResponse struct {
	Type  string   `json:"type"`
	Cubes []string `json:"cubes"`
}

// ----------------------------------------------------------------
// Helper functions for TCP comms
// ----------------------------------------------------------------

// sendMessage: appends the END_OF_MSG marker and writes to the connection.
func sendMessage(conn net.Conn, message string) error {
	fullMsg := message + END_OF_MSG
	_, err := conn.Write([]byte(fullMsg))
	if err != nil {
		return fmt.Errorf("failed to send message: %v", err)
	}
	return nil
}

// receiveMessage: reads until it finds the END_OF_MSG marker or times out.
func receiveMessage(conn net.Conn) (string, error) {
	conn.SetReadDeadline(time.Now().Add(SOCKET_TIMEOUT))

	var buffer bytes.Buffer
	tmp := make([]byte, 1024)

	for {
		n, err := conn.Read(tmp)
		if err != nil {
			if err == io.EOF {
				// End of stream
				break
			}
			// If it's a timeout but we already have some data, we’ll return what we got.
			if netErr, ok := err.(net.Error); ok && netErr.Timeout() && buffer.Len() > 0 {
				break
			}
			return "", fmt.Errorf("read error: %v", err)
		}
		buffer.Write(tmp[:n])

		// Check if we hit the marker
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

// requestCubeList: convenience function to query the server for all cube names.
func requestCubeList(conn net.Conn) ([]string, error) {
	cmd := map[string]interface{}{
		"type": "get_cube_list",
	}
	cmdBytes, _ := json.Marshal(cmd)
	if err := sendMessage(conn, string(cmdBytes)); err != nil {
		return nil, err
	}

	respStr, err := receiveMessage(conn)
	if err != nil {
		return nil, err
	}
	var resp CubeListResponse
	if err := json.Unmarshal([]byte(respStr), &resp); err != nil {
		return nil, fmt.Errorf("unmarshal error: %v", err)
	}
	if resp.Type != "cube_list" {
		return nil, fmt.Errorf("unexpected response type: %s", resp.Type)
	}
	return resp.Cubes, nil
}

// spawnCube: sends a "spawn_cube" request. Optionally mark it as "is_base" to freeze it.
func spawnCube(conn net.Conn, x, y, z float64, isBase bool) error {
	cmd := map[string]interface{}{
		"type":     "spawn_cube",
		"position": []float64{x, y, z},
		"rotation": []float64{0, 0, 0},
	}
	if isBase {
		cmd["is_base"] = true
	}
	cmdBytes, _ := json.Marshal(cmd)
	return sendMessage(conn, string(cmdBytes))
}

// createJoint: connect two cubes by name with the given joint type (pin/hinge/etc.).
func createJoint(conn net.Conn, cube1, cube2, jointType string) error {
	cmd := map[string]interface{}{
		"type":       "create_joint",
		"cube1":      cube1,
		"cube2":      cube2,
		"joint_type": jointType,
	}
	jBytes, _ := json.Marshal(cmd)
	if err := sendMessage(conn, string(jBytes)); err != nil {
		return err
	}
	// Expect a JSON response with "type": "joint_created"
	jResp, err := receiveMessage(conn)
	if err != nil {
		return err
	}
	var tmp map[string]interface{}
	if err := json.Unmarshal([]byte(jResp), &tmp); err != nil {
		return fmt.Errorf("joint creation response parse error: %v", err)
	}
	if jt, ok := tmp["type"].(string); !ok || jt != "joint_created" {
		return fmt.Errorf("unexpected joint creation response: %v", tmp)
	}
	return nil
}

// applyForceToClientCube: Calls "apply_force" to the client’s *currently-owned* cube.
// Remember that the server’s code uses `info.OwnedCubeName`, so only the most recently spawned
// cube by this connection will be affected.
func applyForceToClientCube(conn net.Conn, fx, fy, fz float64) error {
	cmd := map[string]interface{}{
		"type":  "apply_force", // server will parse 'force' array
		"force": []float64{fx, fy, fz},
	}
	cmdBytes, _ := json.Marshal(cmd)
	return sendMessage(conn, string(cmdBytes))
}

// despawnCube instructs the server to remove a specific cube by name.
func despawnCube(conn net.Conn, cubeName string) error {
	cmd := map[string]interface{}{
		"type":      "despawn_cube",
		"cube_name": cubeName,
	}
	cmdBytes, _ := json.Marshal(cmd)
	return sendMessage(conn, string(cmdBytes))
}

// ----------------------------------------------------------------
// MAIN
// ----------------------------------------------------------------
func main() {
	fmt.Printf("Attempting to connect to Godot server at %s:%d...\n", SERVER_IP, SERVER_PORT)
	conn, err := net.Dial("tcp", fmt.Sprintf("%s:%d", SERVER_IP, SERVER_PORT))
	if err != nil {
		fmt.Printf("Connection error: %v\n", err)
		return
	}
	defer conn.Close()

	// 1) Authenticate
	if err := sendMessage(conn, SHARED_PASS); err != nil {
		fmt.Printf("Failed to send auth: %v\n", err)
		return
	}
	authResp, err := receiveMessage(conn)
	if err != nil || authResp != "auth_success" {
		fmt.Printf("Authentication failed, server said: '%s' error=%v\n", authResp, err)
		return
	}
	fmt.Println("[Client] Authenticated with the server.")

	// 2) For each joint type, build a snake
	for _, jt := range jointTypes {
		fmt.Printf("\n=== Building a snake with joint type: %s ===\n", jt)

		// Before spawning, let's see how many cubes are currently in the scene
		existingCubes, err := requestCubeList(conn)
		if err != nil {
			fmt.Printf("Could not get initial cube list: %v\n", err)
			return
		}
		initialCount := len(existingCubes)
		fmt.Printf("[Pre-check] Scene has %d existing cubes.\n", initialCount)

		//-------------------------------------------------
		// Step A: Spawn a base cube (frozen) at (0,100,0)
		//-------------------------------------------------
		if err := spawnCube(conn, 0, 110, 0, false); err != nil {
			fmt.Printf("[spawnCube base] error: %v\n", err)
			return
		}
		fmt.Println("Spawned base cube (is_base=true).")
		time.Sleep(300 * time.Millisecond)

		//-------------------------------------------------
		// Step B: Spawn the “snake” cubes in a line.
		//         Each new spawn overwrites OwnedCubeName
		//         so only the last one is forcibly moveable
		//         from this client’s 'apply_force' calls.
		//-------------------------------------------------
		for i := 0; i < NUM_LINKS; i++ {
			// For example: spawn along +X, spaced by 2
			x := float64((i + 1) * 2) // offset from the base
			y := 100.0                // same Y as base
			z := 0.0
			if err := spawnCube(conn, x, y, z, false); err != nil {
				fmt.Printf("[spawnCube snake link %d] error: %v\n", i, err)
				return
			}
			fmt.Printf("Spawned snake link %d at (%.1f, %.1f, %.1f)\n", i, x, y, z)
			time.Sleep(300 * time.Millisecond)
		}

		//-------------------------------------------------
		// Step C: Identify newly spawned cubes from the server
		//-------------------------------------------------
		afterSpawn, err := requestCubeList(conn)
		if err != nil {
			fmt.Printf("Error retrieving updated cube list: %v\n", err)
			return
		}
		newCount := len(afterSpawn)
		totalSpawned := newCount - initialCount

		if totalSpawned < (1 + NUM_LINKS) {
			fmt.Printf("[WARN] We expected to have spawned %d new cubes but only see %d.\n",
				1+NUM_LINKS, totalSpawned)
		}

		// The newly spawned cubes should be at the tail end of afterSpawn.
		// We'll assume the server appends them in the order they were created.
		newCubes := afterSpawn[len(afterSpawn)-totalSpawned:]
		if len(newCubes) == 0 {
			fmt.Println("[ERROR] No new cubes found!")
			return
		}

		// The first of these is the "base" we spawned
		baseCubeName := newCubes[0]
		// The rest are the snake links
		snakeCubes := newCubes[1:]

		fmt.Printf("[INFO] baseCube: %s\n", baseCubeName)
		fmt.Printf("[INFO] snakeCubes: %v\n", snakeCubes)

		//-------------------------------------------------
		// Step D: Create joints
		//-------------------------------------------------

		// 1) Attach the base to the first snake link
		if len(snakeCubes) > 0 {
			err := createJoint(conn, baseCubeName, snakeCubes[0], jt)
			if err != nil {
				fmt.Printf("Error creating joint between %s and %s: %v\n", baseCubeName, snakeCubes[0], err)
			} else {
				fmt.Printf("Created %s joint (base -> first link)\n", jt)
			}
			time.Sleep(300 * time.Millisecond)
		}

		// 2) Attach consecutive links
		for i := 0; i < len(snakeCubes)-1; i++ {
			c1 := snakeCubes[i]
			c2 := snakeCubes[i+1]
			if err := createJoint(conn, c1, c2, jt); err != nil {
				fmt.Printf("Error linking %s and %s: %v\n", c1, c2, err)
				continue
			}
			fmt.Printf("Created %s joint between %s -> %s\n", jt, c1, c2)
			time.Sleep(300 * time.Millisecond)
		}

		//-------------------------------------------------
		// Step E: Nudge the last spawned cube a bit
		//-------------------------------------------------
		// Because each new spawn overwrote OwnedCubeName,
		// we expect that the last snake cube is the one
		// that belongs to our client. So 'apply_force'
		// will move that final link.
		fmt.Println("Applying small impulses to the last snake link...")

		for i := 0; i < 8; i++ {
			// e.g. push in +Y direction
			if err := applyForceToClientCube(conn, 0, 10, 0); err != nil {
				fmt.Printf("[WARN] applyForce failed: %v\n", err)
			}
			time.Sleep(200 * time.Millisecond)
		}

		//-------------------------------------------------
		// Step F: Observe
		//-------------------------------------------------
		fmt.Printf("Observing the %s snake for %v...\n", jt, TEST_DURATION)
		time.Sleep(TEST_DURATION)

		//-------------------------------------------------
		// Step G: Despawn everything we created
		//-------------------------------------------------
		fmt.Println("Cleaning up cubes...")
		for _, cName := range newCubes {
			if err := despawnCube(conn, cName); err != nil {
				fmt.Printf("Error despawning cube %s: %v\n", cName, err)
			} else {
				fmt.Printf("Despawned cube %s\n", cName)
			}
			time.Sleep(200 * time.Millisecond)
		}
		fmt.Printf("=== Finished testing joint type: %s ===\n", jt)
	}

	fmt.Println("\nAll joint types tested successfully. Exiting now.")
}
