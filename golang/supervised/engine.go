package main

import (
	"encoding/csv"
	"encoding/json"
	"fmt"
	"math"
	"math/rand"
	"net"
	"os"
	"strings"
	"time"

	paragon "github.com/OpenFluke/PARAGON" // Make sure this import is correct.
)

// ---------------------------
// Section 1: Data Generation
// ---------------------------

// generateTrajectoryData mimics the Python version.
func generateTrajectoryData(startPos, endPos []float64, numPoints int) ([][]float64, [][]float64, [][]float64) {
	positions := make([][]float64, numPoints)
	velocities := make([][]float64, numPoints)
	forces := make([][]float64, numPoints)

	currentPos := make([]float64, len(startPos))
	copy(currentPos, startPos)
	currentVel := []float64{0.0, 0.0, 0.0}
	dt := 0.1
	gravity := []float64{0, -9.81, 0}
	maxForce := 1.5
	target := make([]float64, len(endPos))
	copy(target, endPos)

	for i := 0; i < numPoints; i++ {
		positions[i] = make([]float64, 3)
		velocities[i] = make([]float64, 3)
		forces[i] = make([]float64, 3)
		copy(positions[i], currentPos)
		copy(velocities[i], currentVel)

		// Compute error and distance.
		error := make([]float64, 3)
		for j := 0; j < 3; j++ {
			error[j] = target[j] - currentPos[j]
		}
		distance := math.Sqrt(error[0]*error[0] + error[1]*error[1] + error[2]*error[2])

		var gainX, gainY, gainZ float64
		if distance < 20 {
			gainX = 0.1
			gainY = 0.05
			gainZ = -0.5
		} else {
			gainX = 0.3
			gainY = 0.1
			gainZ = -2.0
		}

		force := make([]float64, 3)
		// Use clip (simulate numpy.clip)
		force[0] = math.Max(-maxForce, math.Min(maxForce, error[0]*gainX))
		// Add small offset for gravity compensation.
		force[1] = math.Max(-maxForce, math.Min(maxForce, 0.981+error[1]*gainY))
		force[2] = math.Max(-maxForce, math.Min(0.0, error[2]*gainZ))

		// Cap force magnitude if necessary.
		forceMagnitude := math.Sqrt(force[0]*force[0] + force[1]*force[1] + force[2]*force[2])
		if forceMagnitude > maxForce {
			for j := 0; j < 3; j++ {
				force[j] = (force[j] / forceMagnitude) * maxForce
			}
		}
		copy(forces[i], force)

		// Update physics
		acceleration := make([]float64, 3)
		for j := 0; j < 3; j++ {
			acceleration[j] = force[j] + gravity[j]
			currentVel[j] += acceleration[j] * dt
			currentPos[j] += currentVel[j] * dt
		}
	}

	// Ensure last state is exactly the target.
	positions[numPoints-1] = endPos
	velocities[numPoints-1] = []float64{0, 0, 0}
	forces[numPoints-1] = []float64{0, 0, 0}

	return positions, velocities, forces
}

// saveToCSV writes the data with proper column headers.
func saveToCSV(filename string, positions, velocities, forces [][]float64) error {
	file, err := os.Create(filename)
	if err != nil {
		return err
	}
	defer file.Close()

	writer := csv.NewWriter(file)
	defer writer.Flush()

	// Write header.
	if err := writer.Write([]string{"pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z", "force_x", "force_y", "force_z"}); err != nil {
		return err
	}

	// Write each row.
	for i := 0; i < len(positions); i++ {
		row := []string{
			fmt.Sprintf("%f", positions[i][0]),
			fmt.Sprintf("%f", positions[i][1]),
			fmt.Sprintf("%f", positions[i][2]),
			fmt.Sprintf("%f", velocities[i][0]),
			fmt.Sprintf("%f", velocities[i][1]),
			fmt.Sprintf("%f", velocities[i][2]),
			fmt.Sprintf("%f", forces[i][0]),
			fmt.Sprintf("%f", forces[i][1]),
			fmt.Sprintf("%f", forces[i][2]),
		}
		if err := writer.Write(row); err != nil {
			return err
		}
	}
	return nil
}

// -----------------------------
// Section 2: Training and Evaluation
// -----------------------------

// splitData splits data indices into train and test sets.
func splitData(positions, velocities, forces [][]float64, testSize float64) ([][]float64, [][]float64, [][]float64, [][]float64) {
	numSamples := len(positions)
	trainSize := int(float64(numSamples) * (1.0 - testSize))
	indices := rand.Perm(numSamples)

	X := make([][]float64, numSamples)
	y := make([][]float64, numSamples)
	for i := 0; i < numSamples; i++ {
		// Features: position and velocity (6 values).
		X[i] = append(positions[i], velocities[i]...)
		// Targets: forces (3 values).
		y[i] = forces[i]
	}

	XTrain := make([][]float64, trainSize)
	yTrain := make([][]float64, trainSize)
	XTest := make([][]float64, numSamples-trainSize)
	yTest := make([][]float64, numSamples-trainSize)
	for i, idx := range indices {
		if i < trainSize {
			XTrain[i] = X[idx]
			yTrain[i] = y[idx]
		} else {
			XTest[i-trainSize] = X[idx]
			yTest[i-trainSize] = y[idx]
		}
	}
	return XTrain, yTrain, XTest, yTest
}

// trainNetwork builds and trains a neural network with architecture:
// Input: 6, Hidden1: 200, Hidden2: 150, Hidden3: 100, Output: 3.
// Training is done sample-by-sample.
func trainNetwork(positions, velocities, forces [][]float64) (*paragon.Network, error) {
	numSamples := len(positions)
	X := make([][]float64, numSamples)
	y := make([][]float64, numSamples)
	for i := 0; i < numSamples; i++ {
		X[i] = append(positions[i], velocities[i]...)
		y[i] = forces[i]
	}

	// Define fixed layer sizes (each sample is 1xN).
	layerSizes := []struct{ Width, Height int }{
		{6, 1},   // Input layer: 6 features.
		{200, 1}, // Hidden layer 1: 200 neurons.
		{150, 1}, // Hidden layer 2: 150 neurons.
		{100, 1}, // Hidden layer 3: 100 neurons.
		{3, 1},   // Output layer: 3 outputs.
	}
	activations := []string{"linear", "relu", "relu", "relu", "linear"}
	fullyConnected := []bool{true, true, true, true, true}

	net := paragon.NewNetwork(layerSizes, activations, fullyConnected)

	numEpochs := 8
	learningRate := 0.0005

	for epoch := 0; epoch < numEpochs; epoch++ {
		totalLoss := 0.0
		for i := 0; i < numSamples; i++ {
			// Each sample is wrapped as a 2D slice.
			input := [][]float64{X[i]}
			target := [][]float64{y[i]}
			net.Forward(input)
			loss := net.ComputeLoss(target)
			totalLoss += loss
			net.Backward(target, learningRate)
		}
		if epoch%1 == 0 {
			fmt.Printf("Epoch %d, Loss: %f\n", epoch, totalLoss/float64(numSamples))
		}
	}

	return net, nil
}

// evaluateNetwork computes an R² score over test data.
func evaluateNetwork(net *paragon.Network, XTest, yTest [][]float64) float64 {
	if len(XTest) == 0 {
		return 0.0
	}

	predictions := make([][]float64, len(XTest))
	for i := 0; i < len(XTest); i++ {
		input := [][]float64{XTest[i]}
		net.Forward(input)
		outputLayer := net.Layers[net.OutputLayer]
		output := make([]float64, outputLayer.Width)
		for x := 0; x < outputLayer.Width; x++ {
			output[x] = outputLayer.Neurons[0][x].Value
		}
		predictions[i] = output
	}

	var ssTot, ssRes float64
	for dim := 0; dim < 3; dim++ {
		meanY := 0.0
		for i := 0; i < len(yTest); i++ {
			meanY += yTest[i][dim]
		}
		meanY /= float64(len(yTest))
		ssTotDim := 0.0
		ssResDim := 0.0
		for i := 0; i < len(yTest); i++ {
			diff := yTest[i][dim] - meanY
			ssTotDim += diff * diff
			predDiff := predictions[i][dim] - yTest[i][dim]
			ssResDim += predDiff * predDiff
		}
		if ssTotDim != 0 {
			r2Dim := 1.0 - (ssResDim / ssTotDim)
			ssTot += ssTotDim
			ssRes += ssResDim * r2Dim
		}
	}
	if ssTot == 0 {
		return 0.0
	}
	return 1.0 - (ssRes / ssTot)
}

// -------------------------
// Section 3: Cube Control
// -------------------------

const (
	SERVER_IP       = "192.168.0.116"
	SERVER_PORT     = 14000
	SHARED_PASSWORD = "my_secure_password"
	END_OF_MSG      = "<???DONE???---"
)

func sendMessage(conn net.Conn, message string) error {
	fullMessage := message + END_OF_MSG
	_, err := conn.Write([]byte(fullMessage))
	return err
}

func receiveMessage(conn net.Conn) (string, error) {
	conn.SetReadDeadline(time.Now().Add(1 * time.Second))
	buffer := make([]byte, 1024)
	var response string
	for {
		n, err := conn.Read(buffer)
		if err != nil {
			if ne, ok := err.(net.Error); ok && ne.Timeout() {
				break
			}
			return "", err
		}
		response += string(buffer[:n])
		if strings.Contains(response, END_OF_MSG) {
			response = strings.Replace(response, END_OF_MSG, "", -1)
			break
		}
	}
	return strings.TrimSpace(response), nil
}

func spawnAndControlCube(cubeID int, startPosition, targetPosition []float64, network *paragon.Network) {
	conn, err := net.Dial("tcp", fmt.Sprintf("%s:%d", SERVER_IP, SERVER_PORT))
	if err != nil {
		fmt.Printf("[Cube %d] Connection error: %v\n", cubeID, err)
		return
	}
	defer conn.Close()

	fmt.Printf("[Cube %d] Connected to server.\n", cubeID)
	if err := sendMessage(conn, SHARED_PASSWORD); err != nil {
		fmt.Printf("[Cube %d] Send error: %v\n", cubeID, err)
		return
	}
	authResponse, err := receiveMessage(conn)
	if err != nil {
		fmt.Printf("[Cube %d] Receive error: %v\n", cubeID, err)
		return
	}
	fmt.Printf("[Cube %d] Authentication response: %v\n", cubeID, authResponse)
	if authResponse != "auth_success" {
		fmt.Printf("[Cube %d] Authentication failed.\n", cubeID)
		return
	}

	spawnCubeCmd := map[string]interface{}{
		"type":     "spawn_cube",
		"position": startPosition,
		"rotation": []float64{0, 0, 0},
	}
	jsonCmd, _ := json.Marshal(spawnCubeCmd)
	if err := sendMessage(conn, string(jsonCmd)); err != nil {
		fmt.Printf("[Cube %d] Spawn error: %v\n", cubeID, err)
		return
	}
	fmt.Printf("[Cube %d] Spawned at %v\n", cubeID, startPosition)

	// Use a simple proportional controller for cube movement.
	k := 0.05
	maxForceMagnitude := 2.0
	target := make([]float64, len(targetPosition))
	copy(target, targetPosition)
	tolerance := 5.0

	startTime := time.Now()
	for time.Since(startTime).Seconds() < 10 {
		if err := sendMessage(conn, `{"type": "get_cube_state"}`); err != nil {
			fmt.Printf("[Cube %d] State request error: %v\n", cubeID, err)
			continue
		}
		stateResponse, err := receiveMessage(conn)
		if err != nil {
			fmt.Printf("[Cube %d] State receive error: %v\n", cubeID, err)
			continue
		}
		var state map[string]interface{}
		if err := json.Unmarshal([]byte(stateResponse), &state); err != nil {
			fmt.Printf("[Cube %d] JSON decode error: %v\n", cubeID, err)
			continue
		}
		pos := make([]float64, 3)
		vel := make([]float64, 3)
		if posData, ok := state["position"].([]interface{}); ok {
			for i := 0; i < 3; i++ {
				if v, ok := posData[i].(float64); ok {
					pos[i] = v
				}
			}
		}
		if velData, ok := state["linear_velocity"].([]interface{}); ok {
			for i := 0; i < 3; i++ {
				if v, ok := velData[i].(float64); ok {
					vel[i] = v
				}
			}
		}

		// Compute proportional force.
		errorVec := make([]float64, 3)
		for i := 0; i < 3; i++ {
			errorVec[i] = target[i] - pos[i]
		}
		forceTotal := make([]float64, 3)
		for i := 0; i < 3; i++ {
			forceTotal[i] = k * errorVec[i]
		}
		forceMag := math.Sqrt(forceTotal[0]*forceTotal[0] + forceTotal[1]*forceTotal[1] + forceTotal[2]*forceTotal[2])
		if forceMag > maxForceMagnitude {
			for i := 0; i < 3; i++ {
				forceTotal[i] = (forceTotal[i] / forceMag) * maxForceMagnitude
			}
		}
		distanceToTarget := math.Sqrt(errorVec[0]*errorVec[0] + errorVec[1]*errorVec[1] + errorVec[2]*errorVec[2])
		if distanceToTarget < tolerance {
			fmt.Printf("[Cube %d] Near target, stopping. Position: %v\n", cubeID, pos)
			break
		}

		applyForceCmd := map[string]interface{}{
			"type":  "apply_force",
			"force": forceTotal,
		}
		jsonCmd, _ = json.Marshal(applyForceCmd)
		if err := sendMessage(conn, string(jsonCmd)); err != nil {
			fmt.Printf("[Cube %d] Apply force error: %v\n", cubeID, err)
			continue
		}
		fmt.Printf("[Cube %d] Pos: %v, Vel: %v, Force: %v\n", cubeID, pos, vel, forceTotal)
		time.Sleep(100 * time.Millisecond)
	}
	fmt.Printf("[Cube %d] Control loop completed.\n", cubeID)
}

func main() {
	rand.Seed(time.Now().UnixNano())

	// Define start and end positions.
	startPos := []float64{0, 100, 40}
	endPos := []float64{69.58438552780794, 72.56208670917846, 30.29450219308874}

	// Generate trajectory data.
	positions, velocities, forces := generateTrajectoryData(startPos, endPos, 100)
	if err := saveToCSV("cube_trajectory_data_properly_fixed.csv", positions, velocities, forces); err != nil {
		fmt.Printf("Error saving CSV: %v\n", err)
		return
	}
	fmt.Println("Data saved to 'cube_trajectory_data_properly_fixed.csv'")

	// Split data into training and test sets.
	_, _, XTest, yTest := splitData(positions, velocities, forces, 0.2)

	// Train the network.
	net, err := trainNetwork(positions, velocities, forces)
	if err != nil {
		fmt.Printf("Error training network: %v\n", err)
		return
	}

	// Evaluate the model.
	r2 := evaluateNetwork(net, XTest, yTest)
	fmt.Printf("Model R^2 Score: %f\n", r2)

	// Save the trained model (using JSON serialization here).
	if err := net.SaveToJSON("cube_force_model_properly_fixed.json"); err != nil {
		fmt.Printf("Error saving model: %v\n", err)
	} else {
		fmt.Println("Model saved as 'cube_force_model_properly_fixed.json'")
	}

	// Spawn and control the cube.
	spawnAndControlCube(1, startPos, endPos, net)
}
