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
// Adjust these to match your environment
// ----------------------------------------------------------------
const (
	SERVER_IP      = "127.0.0.1"
	SERVER_PORT    = 14000
	SHARED_PASS    = "my_secure_password"
	END_OF_MSG     = "<???DONE???---"
	SOCKET_TIMEOUT = 2 * time.Second

	NUM_CUBES     = 6               // number of cubes per "snake"
	TEST_DURATION = 5 * time.Second // how long we watch each snake
)

// We'll test all 5 types
var testTypes = []string{"pin", "hinge", "slider", "conetwist", "bone"}

// ----------------------------------------------------------------
// For parsing the server's "get_cube_list" response
// ----------------------------------------------------------------
type CubeListResponse struct {
	Type  string   `json:"type"`
	Cubes []string `json:"cubes"`
}

// For parsing "joint_created" / "bone_created" server responses
type JointCreationResponse struct {
	Type      string `json:"type"`
	JointName string `json:"joint_name"`
}

// ----------------------------------------------------------------
// Utility: send a message with the END_OF_MSG marker
// ----------------------------------------------------------------
func sendMessage(conn net.Conn, message string) error {
	fullMsg := message + END_OF_MSG
	_, err := conn.Write([]byte(fullMsg))
	return err
}

// ----------------------------------------------------------------
// Utility: read until we find END_OF_MSG or time out
// ----------------------------------------------------------------
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
			// If we timed out but have partial data, parse it
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
	if idx := strings.Index(result, END_OF_MSG); idx >= 0 {
		result = result[:idx]
	}
	return strings.TrimSpace(result), nil
}

// ----------------------------------------------------------------
// MAIN
// ----------------------------------------------------------------
func main() {
	fmt.Printf("Connecting to server at %s:%d...\n", SERVER_IP, SERVER_PORT)
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
		fmt.Printf("Authentication failed. Server said '%s' (err=%v)\n", authResp, err)
		return
	}
	fmt.Println("Authenticated successfully!")

	// 2) For each joint/bone type, build & test a “snake”
	for _, t := range testTypes {
		fmt.Printf("\n=== Testing type: %s ===\n", t)

		// Step A: Spawn cubes
		for i := 0; i < NUM_CUBES; i++ {
			spawnPos := []float64{float64(i * 2), 100, 40} // each 2 units apart along X
			spawnCmd := map[string]interface{}{
				"type":     "spawn_cube",
				"position": spawnPos,
				"rotation": []float64{0, 0, 0},
			}
			if err := sendCommand(conn, spawnCmd); err != nil {
				fmt.Printf("Error spawning cube %d: %v\n", i, err)
				return
			}
			time.Sleep(100 * time.Millisecond)
		}

		// Wait for server to create them
		time.Sleep(1 * time.Second)

		// Step B: Get list of cubes (the last N are ours)
		if err := sendCommand(conn, map[string]interface{}{"type": "get_cube_list"}); err != nil {
			fmt.Printf("Error requesting cube list: %v\n", err)
			return
		}
		listRespStr, err := receiveMessage(conn)
		if err != nil {
			fmt.Printf("Error reading cube list: %v\n", err)
			return
		}
		var cubesList CubeListResponse
		if err := json.Unmarshal([]byte(listRespStr), &cubesList); err != nil {
			fmt.Printf("Error unmarshaling cube list: %v\n", err)
			return
		}
		if cubesList.Type != "cube_list" || len(cubesList.Cubes) < NUM_CUBES {
			fmt.Printf("Invalid cube list response: %+v\n", cubesList)
			return
		}
		snakeCubes := cubesList.Cubes[len(cubesList.Cubes)-NUM_CUBES:]
		fmt.Printf("Our %s snake cubes: %v\n", t, snakeCubes)

		// Step C: Link consecutive cubes
		var createdJointNames []string // store joint names to tweak parameters

		for i := 0; i < len(snakeCubes)-1; i++ {
			cA := snakeCubes[i]
			cB := snakeCubes[i+1]

			var createCmd map[string]interface{}
			if t == "bone" {
				createCmd = map[string]interface{}{
					"type":  "create_bone",
					"cube1": cA,
					"cube2": cB,
				}
			} else {
				createCmd = map[string]interface{}{
					"type":       "create_joint",
					"cube1":      cA,
					"cube2":      cB,
					"joint_type": t,
				}
			}

			if err := sendCommand(conn, createCmd); err != nil {
				fmt.Printf("create link error: %v\n", err)
				continue
			}

			// Read the response (joint_created or bone_created)
			linkRespStr, err := receiveMessage(conn)
			if err != nil {
				fmt.Printf("Error reading link creation response: %v\n", err)
				continue
			}
			var linkRes JointCreationResponse
			if err := json.Unmarshal([]byte(linkRespStr), &linkRes); err != nil {
				fmt.Printf("Error decoding creation response: %v\n", err)
				continue
			}

			if t == "bone" && linkRes.Type == "bone_created" {
				fmt.Printf("[Client] Created bone between %s & %s => name=%s\n", cA, cB, linkRes.JointName)
				// We can treat that as a “joint name” if we wanted to set param, but currently
				// we only set param on “joint_created” – so do nothing special here
			} else if t != "bone" && linkRes.Type == "joint_created" {
				fmt.Printf("[Client] Created joint '%s' for %s => name=%s\n", t, cA+"->"+cB, linkRes.JointName)
				// store the joint name so we can set params
				createdJointNames = append(createdJointNames, linkRes.JointName)
			} else {
				fmt.Printf("Unexpected creation response: %s\n", linkRes.Type)
			}

			time.Sleep(100 * time.Millisecond)
		}

		// Step C2: Optionally set joint parameters for pin or hinge or slider
		// For example, if it's pin: set "bias", "damping" to 1.0, "impulse_clamp"=0
		// If it's hinge: set "limit_upper=45" or "limit_softness=1.0" etc.

		switch t {
		case "pin":
			// For each created pin joint, set some stiffness
			for _, jointName := range createdJointNames {
				// Example: set "bias"=1
				setParamCmd := map[string]interface{}{
					"type":       "set_joint_param",
					"joint_name": jointName,
					"param_name": "bias",
					"value":      1.0,
				}
				_ = sendCommand(conn, setParamCmd)
				time.Sleep(100 * time.Millisecond)

				// set "damping"=1
				setParamCmd = map[string]interface{}{
					"type":       "set_joint_param",
					"joint_name": jointName,
					"param_name": "damping",
					"value":      1.0,
				}
				_ = sendCommand(conn, setParamCmd)
				time.Sleep(100 * time.Millisecond)

				// set "impulse_clamp"=0
				setParamCmd = map[string]interface{}{
					"type":       "set_joint_param",
					"joint_name": jointName,
					"param_name": "impulse_clamp",
					"value":      0.0,
				}
				_ = sendCommand(conn, setParamCmd)
				time.Sleep(100 * time.Millisecond)

				fmt.Printf("Set stiff params on pin joint '%s'\n", jointName)
			}

		case "hinge":
			// For each hinge, set upper limit=45, softness=1, bias=1, relaxation=1
			for _, jointName := range createdJointNames {
				// limit_upper=45
				setParamCmd := map[string]interface{}{
					"type":       "set_joint_param",
					"joint_name": jointName,
					"param_name": "limit_upper",
					"value":      45.0,
				}
				_ = sendCommand(conn, setParamCmd)
				time.Sleep(100 * time.Millisecond)

				// limit_softness=1
				setParamCmd = map[string]interface{}{
					"type":       "set_joint_param",
					"joint_name": jointName,
					"param_name": "limit_softness",
					"value":      1.0,
				}
				_ = sendCommand(conn, setParamCmd)
				time.Sleep(100 * time.Millisecond)

				// limit_bias=1
				setParamCmd = map[string]interface{}{
					"type":       "set_joint_param",
					"joint_name": jointName,
					"param_name": "limit_bias",
					"value":      1.0,
				}
				_ = sendCommand(conn, setParamCmd)
				time.Sleep(100 * time.Millisecond)

				// limit_relaxation=1
				setParamCmd = map[string]interface{}{
					"type":       "set_joint_param",
					"joint_name": jointName,
					"param_name": "limit_relaxation",
					"value":      1.0,
				}
				_ = sendCommand(conn, setParamCmd)
				time.Sleep(100 * time.Millisecond)

				fmt.Printf("Set stiff hinge params on hinge '%s'\n", jointName)
			}

		case "slider":
			// For slider, you could do "linear_limit_lower" or "linear_limit_upper", etc.
			// e.g. set an upper limit=5
			for _, jointName := range createdJointNames {
				setParamCmd := map[string]interface{}{
					"type":       "set_joint_param",
					"joint_name": jointName,
					"param_name": "linear_limit_upper",
					"value":      5.0,
				}
				_ = sendCommand(conn, setParamCmd)
				time.Sleep(100 * time.Millisecond)
				fmt.Printf("Set upper limit=5 on slider '%s'\n", jointName)
			}

		case "conetwist":
			// similarly "swing_span", "twist_span", "bias", "softness", etc.
			for _, jointName := range createdJointNames {
				setParamCmd := map[string]interface{}{
					"type":       "set_joint_param",
					"joint_name": jointName,
					"param_name": "swing_span",
					"value":      30.0,
				}
				_ = sendCommand(conn, setParamCmd)
				time.Sleep(100 * time.Millisecond)

				setParamCmd = map[string]interface{}{
					"type":       "set_joint_param",
					"joint_name": jointName,
					"param_name": "twist_span",
					"value":      60.0,
				}
				_ = sendCommand(conn, setParamCmd)
				time.Sleep(100 * time.Millisecond)

				setParamCmd = map[string]interface{}{
					"type":       "set_joint_param",
					"joint_name": jointName,
					"param_name": "bias",
					"value":      1.0,
				}
				_ = sendCommand(conn, setParamCmd)
				time.Sleep(100 * time.Millisecond)

				fmt.Printf("Set conetwist params on joint '%s'\n", jointName)
			}

		case "bone":
			// "bone" isn't physically a separate joint type on the server, so no param calls here
		}

		// Step D: Apply force to the head cube
		headCube := snakeCubes[0]
		for i := 0; i < 10; i++ {
			forceCmd := map[string]interface{}{
				"type":      "apply_force",
				"cube_name": headCube,
				"force":     []float64{10, 0, 0},
			}
			_ = sendCommand(conn, forceCmd)
			fmt.Printf("Applied force to %s\n", headCube)
			time.Sleep(200 * time.Millisecond)
		}

		// Step E: observe
		fmt.Printf("[Client] Observing %s snake for %v...\n", t, TEST_DURATION)
		time.Sleep(TEST_DURATION)

		// Step F: despawn
		for _, c := range snakeCubes {
			delCmd := map[string]interface{}{
				"type":      "despawn_cube",
				"cube_name": c,
			}
			_ = sendCommand(conn, delCmd)
			time.Sleep(100 * time.Millisecond)
			fmt.Printf("Despawned cube %s\n", c)
		}
		fmt.Printf("=== Done with %s ===\n", t)
	}

	fmt.Println("All joint/bone tests completed.")
}

// sendCommand: marshals the cmd to JSON, calls sendMessage
func sendCommand(conn net.Conn, cmd map[string]interface{}) error {
	data, err := json.Marshal(cmd)
	if err != nil {
		return fmt.Errorf("json marshal error: %v", err)
	}
	return sendMessage(conn, string(data))
}
