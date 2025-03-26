package main

import (
	"bytes"
	"encoding/gob"
	"encoding/json"
	"fmt"
	"io"
	"math"
	"math/rand"
	"net"
	"os"
	"sync"
	"time"

	// Paragon AI framework import
	paragon "github.com/OpenFluke/PARAGON"
)

// =================
// Configuration
// =================

const (
	SERVER_IP      = "192.168.0.116"
	SERVER_PORT    = 14000
	SHARED_PASS    = "my_secure_password"
	END_OF_MSG     = "<???DONE???---"
	SOCKET_TIMEOUT = 1 * time.Second
)

// ======================
// Some Paragon Usage Demo
// ======================

func demoParagonUsage() {
	fmt.Println("=== Paragon Demo: Creating a small network and training on dummy data ===")
	layerSizes := []struct{ Width, Height int }{
		{3, 1}, // input layer
		{2, 1}, // output layer
	}
	activations := []string{"linear", "softmax"}
	fullyConnected := []bool{true, true}

	nn := paragon.NewNetwork(layerSizes, activations, fullyConnected)

	inputs := [][][]float64{
		{{0, 0, 1}},
		{{0, 1, 0}},
		{{1, 0, 0}},
		{{1, 1, 1}},
	}
	targets := [][][]float64{
		{{1, 0}},
		{{0, 1}},
		{{0, 1}},
		{{1, 0}},
	}
	fmt.Println("[Paragon Demo] Starting training on toy data ...")
	nn.Train(inputs, targets, 10, 0.01)

	fmt.Println("[Paragon Demo] Training complete. Checking accuracy ...")
	acc := paragon.ComputeAccuracy(nn, inputs, targets)
	fmt.Printf("[Paragon Demo] Accuracy on training set = %.2f%%\n", acc*100)

	paragon.EvaluateWithADHD(nn, inputs, targets)
	if nn.Performance != nil {
		fmt.Printf("[Paragon Demo] ADHD Score: %.2f, TotalSamples=%d\n",
			nn.Performance.Score, nn.Performance.Total)
	}
	fmt.Println("=== End Paragon Demo ===\n")
}

// ==========
// Utilities
// ==========

func sendMessage(conn net.Conn, message string) error {
	fullMsg := message + END_OF_MSG
	_, err := conn.Write([]byte(fullMsg))
	return err
}

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
			netErr, ok := err.(net.Error)
			if ok && netErr.Timeout() && buffer.Len() > 0 {
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
	return stripDelimiter(result, END_OF_MSG), nil
}

func stripDelimiter(msg, delimiter string) string {
	idx := bytes.Index([]byte(msg), []byte(delimiter))
	if idx >= 0 {
		msg = msg[:idx]
	}
	return msg
}

func scaleVec(vec []float64, alpha float64) []float64 {
	out := make([]float64, len(vec))
	for i := range vec {
		out[i] = vec[i] * alpha
	}
	return out
}

func vecMag(vec []float64) float64 {
	sum := 0.0
	for _, v := range vec {
		sum += v * v
	}
	return math.Sqrt(sum)
}

// ==============
// Sorting Helpers
// ==============

type pair struct {
	val float64
	idx int
}

func quickSortPairs(arr []pair, low, high int) {
	if low < high {
		p := partition(arr, low, high)
		quickSortPairs(arr, low, p-1)
		quickSortPairs(arr, p+1, high)
	}
}

func partition(arr []pair, low, high int) int {
	pivot := arr[high].val
	i := low
	for j := low; j < high; j++ {
		if arr[j].val > pivot {
			arr[i], arr[j] = arr[j], arr[i]
			i++
		}
	}
	arr[i], arr[high] = arr[high], arr[i]
	return i
}

func eye(n int) [][]float64 {
	e := make([][]float64, n)
	for i := 0; i < n; i++ {
		e[i] = make([]float64, n)
		e[i][i] = 1.0
	}
	return e
}

func maxOffDiagSym(A [][]float64) (p, q int, val float64) {
	n := len(A)
	p, q = -1, -1
	val = 0
	for i := 0; i < n; i++ {
		for j := i + 1; j < n; j++ {
			if math.Abs(A[i][j]) > math.Abs(val) {
				val = A[i][j]
				p = i
				q = j
			}
		}
	}
	return
}

func eigenDecomposeSym(mat [][]float64) ([]float64, [][]float64) {
	n := len(mat)
	A := make([][]float64, n)
	for i := range A {
		A[i] = make([]float64, n)
		copy(A[i], mat[i])
	}
	V := eye(n)

	maxIter := 100
	for iter := 0; iter < maxIter; iter++ {
		p, q, _ := maxOffDiagSym(A)
		if p < 0 || q < 0 {
			break
		}
		if math.Abs(A[p][q]) < 1e-12 {
			break
		}
		var phi float64
		if A[p][p] == A[q][q] {
			phi = math.Pi / 4
		} else {
			phi = 0.5 * math.Atan(2.0*A[p][q]/(A[p][p]-A[q][q]))
		}
		c := math.Cos(phi)
		s := math.Sin(phi)

		a_pp := A[p][p]
		a_qq := A[q][q]

		A[p][p] = c*c*a_pp - 2*s*c*A[p][q] + s*s*a_qq
		A[q][q] = s*s*a_pp + 2*s*c*A[p][q] + c*c*a_qq
		A[p][q] = 0
		A[q][p] = 0

		for i := 0; i < n; i++ {
			if i != p && i != q {
				a_ip := A[i][p]
				a_iq := A[i][q]
				A[i][p] = c*a_ip - s*a_iq
				A[p][i] = A[i][p]
				A[i][q] = s*a_ip + c*a_iq
				A[q][i] = A[i][q]
			}
		}
		for i := 0; i < n; i++ {
			v_ip := V[i][p]
			v_iq := V[i][q]
			V[i][p] = c*v_ip - s*v_iq
			V[i][q] = s*v_ip + c*v_iq
		}
	}
	vals := make([]float64, n)
	for i := 0; i < n; i++ {
		vals[i] = A[i][i]
	}
	return vals, V
}

// ========================
// UnsupervisedModel (PCA)
// ========================

type UnsupervisedModel struct {
	NComponents int
	Means       []float64
	Stds        []float64
	Components  [][]float64
	Fitted      bool
}

func (m *UnsupervisedModel) Fit(X [][]float64) error {
	if len(X) == 0 {
		return fmt.Errorf("no data to fit")
	}
	nSamples := len(X)
	nFeatures := len(X[0])

	means := make([]float64, nFeatures)
	stds := make([]float64, nFeatures)
	for _, row := range X {
		for j := 0; j < nFeatures; j++ {
			means[j] += row[j]
		}
	}
	for j := 0; j < nFeatures; j++ {
		means[j] /= float64(nSamples)
	}
	for _, row := range X {
		for j := 0; j < nFeatures; j++ {
			diff := row[j] - means[j]
			stds[j] += diff * diff
		}
	}
	for j := 0; j < nFeatures; j++ {
		stds[j] = math.Sqrt(stds[j]/float64(nSamples) + 1e-12)
	}

	scaled := make([][]float64, nSamples)
	for i := 0; i < nSamples; i++ {
		scaled[i] = make([]float64, nFeatures)
		for j := 0; j < nFeatures; j++ {
			scaled[i][j] = (X[i][j] - means[j]) / stds[j]
		}
	}

	cov := make([][]float64, nFeatures)
	for i := 0; i < nFeatures; i++ {
		cov[i] = make([]float64, nFeatures)
	}
	for _, row := range scaled {
		for i := 0; i < nFeatures; i++ {
			for j := i; j < nFeatures; j++ {
				cov[i][j] += row[i] * row[j]
			}
		}
	}
	for i := 0; i < nFeatures; i++ {
		for j := i; j < nFeatures; j++ {
			cov[i][j] /= float64(nSamples - 1)
			cov[j][i] = cov[i][j]
		}
	}

	eigVals, eigVecs := eigenDecomposeSym(cov)
	pArr := make([]pair, len(eigVals))
	for i, v := range eigVals {
		pArr[i] = pair{val: v, idx: i}
	}
	quickSortPairs(pArr, 0, len(pArr)-1)

	if m.NComponents > nFeatures {
		m.NComponents = nFeatures
	}
	comps := make([][]float64, m.NComponents)
	for i := 0; i < m.NComponents; i++ {
		comps[i] = make([]float64, nFeatures)
		eigIdx := pArr[i].idx
		for j := 0; j < nFeatures; j++ {
			comps[i][j] = eigVecs[j][eigIdx]
		}
	}

	m.Means = means
	m.Stds = stds
	m.Components = comps
	m.Fitted = true

	fmt.Println("[UnsupervisedModel] Model fitted with PCA.")
	return nil
}

func (m *UnsupervisedModel) ReconstructionError(x []float64) (float64, error) {
	if !m.Fitted {
		return 0, fmt.Errorf("model not fitted")
	}
	if len(x) != len(m.Means) {
		return 0, fmt.Errorf("dimension mismatch: got %d, expected %d", len(x), len(m.Means))
	}

	nFeatures := len(x)
	xScaled := make([]float64, nFeatures)
	for j := 0; j < nFeatures; j++ {
		xScaled[j] = (x[j] - m.Means[j]) / m.Stds[j]
	}

	z := make([]float64, m.NComponents)
	for i := 0; i < m.NComponents; i++ {
		sum := 0.0
		for j := 0; j < nFeatures; j++ {
			sum += xScaled[j] * m.Components[i][j]
		}
		z[i] = sum
	}

	xRec := make([]float64, nFeatures)
	for j := 0; j < nFeatures; j++ {
		sum := 0.0
		for i := 0; i < m.NComponents; i++ {
			sum += z[i] * m.Components[i][j]
		}
		xRec[j] = sum
	}

	l2 := 0.0
	for j := 0; j < nFeatures; j++ {
		diff := xScaled[j] - xRec[j]
		l2 += diff * diff
	}
	return math.Sqrt(l2), nil
}

func (m *UnsupervisedModel) Save(modelPath string) error {
	f, err := os.Create(modelPath)
	if err != nil {
		return err
	}
	defer f.Close()
	enc := gob.NewEncoder(f)
	return enc.Encode(m)
}

func (m *UnsupervisedModel) Load(modelPath string) error {
	f, err := os.Open(modelPath)
	if err != nil {
		return err
	}
	defer f.Close()
	dec := gob.NewDecoder(f)
	err = dec.Decode(m)
	if err == nil {
		fmt.Printf("[UnsupervisedModel] Loaded from %s\n", modelPath)
	}
	return err
}

// ==============
// Data Collection
// ==============

func collectStateData(duration float64) [][]float64 {
	var stateData [][]float64

	conn, err := net.Dial("tcp", fmt.Sprintf("%s:%d", SERVER_IP, SERVER_PORT))
	if err != nil {
		fmt.Println("[Data Collection] Dial error:", err)
		return stateData
	}
	defer conn.Close()

	if err := sendMessage(conn, SHARED_PASS); err != nil {
		fmt.Println("[Data Collection] Auth send error:", err)
		return stateData
	}
	msg, err := receiveMessage(conn)
	if err != nil || msg != "auth_success" {
		fmt.Println("[Data Collection] Authentication failed:", msg, err)
		return stateData
	}

	spawnCubeCmd := map[string]interface{}{
		"type":     "spawn_cube",
		"position": []float64{0, 100, 40},
		"rotation": []float64{0, 0, 0},
	}
	spawnBytes, _ := json.Marshal(spawnCubeCmd)
	_ = sendMessage(conn, string(spawnBytes))
	fmt.Println("[Data Collection] Spawned cube for data collection...")

	start := time.Now()
	for time.Since(start).Seconds() < duration {
		getStateCmd := map[string]interface{}{
			"type": "get_cube_state",
		}
		getStateBytes, _ := json.Marshal(getStateCmd)
		_ = sendMessage(conn, string(getStateBytes))

		resp, _ := receiveMessage(conn)
		if len(resp) == 0 {
			time.Sleep(100 * time.Millisecond)
			continue
		}
		var st map[string]interface{}
		if err := json.Unmarshal([]byte(resp), &st); err != nil {
			continue
		}
		posIF, _ := st["position"].([]interface{})
		velIF, _ := st["linear_velocity"].([]interface{})

		pos := []float64{0, 0, 0}
		vel := []float64{0, 0, 0}
		if len(posIF) == 3 {
			for i := 0; i < 3; i++ {
				pos[i], _ = posIF[i].(float64)
			}
		}
		if len(velIF) == 3 {
			for i := 0; i < 3; i++ {
				vel[i], _ = velIF[i].(float64)
			}
		}
		stateVec := append(pos, vel...)
		stateData = append(stateData, stateVec)
		time.Sleep(100 * time.Millisecond)
	}
	fmt.Println("[Data Collection] Completed.")
	return stateData
}

// ==============
// Model Training
// ==============

func trainModelIfNeeded(modelPath string, nComponents int) (*UnsupervisedModel, error) {
	if _, err := os.Stat(modelPath); os.IsNotExist(err) {
		fmt.Println("No PCA model found. Collecting data to train...")
		X := collectStateData(10.0)
		if len(X) == 0 {
			return nil, fmt.Errorf("no data collected")
		}
		model := &UnsupervisedModel{NComponents: nComponents}
		if err := model.Fit(X); err != nil {
			return nil, err
		}
		if err := model.Save(modelPath); err != nil {
			return nil, err
		}
		return model, nil
	} else {
		model := &UnsupervisedModel{}
		if err := model.Load(modelPath); err != nil {
			return nil, err
		}
		return model, nil
	}
}

// ==============
// Cube Control
// ==============

func controlCube(model *UnsupervisedModel, cubeID int, controlDuration float64, errorThreshold float64) {
	conn, err := net.Dial("tcp", fmt.Sprintf("%s:%d", SERVER_IP, SERVER_PORT))
	if err != nil {
		fmt.Printf("[Cube %d] Dial error: %v\n", cubeID, err)
		return
	}
	defer conn.Close()

	if err := sendMessage(conn, SHARED_PASS); err != nil {
		fmt.Printf("[Cube %d] Auth send error: %v\n", cubeID, err)
		return
	}
	msg, err := receiveMessage(conn)
	if err != nil || msg != "auth_success" {
		fmt.Printf("[Cube %d] Authentication failed: %s %v\n", cubeID, msg, err)
		return
	}

	// Spawn the cube at a unique position based on cubeID
	spawnPos := []float64{float64(cubeID * 10), 100, 40}
	spawnCubeCmd := map[string]interface{}{
		"type":     "spawn_cube",
		"position": spawnPos,
		"rotation": []float64{0, 0, 0},
	}
	spawnBytes, _ := json.Marshal(spawnCubeCmd)
	_ = sendMessage(conn, string(spawnBytes))
	fmt.Printf("[Cube %d] Spawned at %v\n", cubeID, spawnPos)

	// Control parameters
	target := []float64{69.58438552780794, 72.56208670917846, 30.29450219308874}
	k := 0.05
	maxForce := 1.0
	tolerance := 5.0
	startTime := time.Now()

	for time.Since(startTime).Seconds() < controlDuration {
		getStateCmd := map[string]interface{}{"type": "get_cube_state"}
		stBytes, _ := json.Marshal(getStateCmd)
		_ = sendMessage(conn, string(stBytes))

		resp, _ := receiveMessage(conn)
		if len(resp) == 0 {
			time.Sleep(100 * time.Millisecond)
			continue
		}
		var st map[string]interface{}
		if err := json.Unmarshal([]byte(resp), &st); err != nil {
			fmt.Printf("[Cube %d] JSON decode error: %v\n", cubeID, err)
			continue
		}
		posIF, _ := st["position"].([]interface{})
		velIF, _ := st["linear_velocity"].([]interface{})

		pos := []float64{0, 0, 0}
		vel := []float64{0, 0, 0}
		if len(posIF) == 3 {
			for i := 0; i < 3; i++ {
				pos[i], _ = posIF[i].(float64)
			}
		}
		if len(velIF) == 3 {
			for i := 0; i < 3; i++ {
				vel[i], _ = velIF[i].(float64)
			}
		}
		stateVec := append(pos, vel...)

		re, err := model.ReconstructionError(stateVec)
		if err != nil {
			fmt.Printf("[Cube %d] ReconstructionError: %v\n", cubeID, err)
			continue
		}
		fmt.Printf("[Cube %d] Reconstruction error = %.3f\n", cubeID, re)

		errVec := make([]float64, 3)
		for i := 0; i < 3; i++ {
			errVec[i] = target[i] - pos[i]
		}
		forceTotal := scaleVec(errVec, k)

		if re > errorThreshold {
			fmt.Printf("[Cube %d] Novel state detected! Halving force.\n", cubeID)
			forceTotal = scaleVec(forceTotal, 0.5)
		}

		mag := vecMag(forceTotal)
		if mag > maxForce {
			forceTotal = scaleVec(forceTotal, maxForce/mag)
		}

		dist := vecMag(errVec)
		if dist < tolerance {
			fmt.Printf("[Cube %d] Near target, stopping. Position: %v\n", cubeID, pos)
			break
		}

		applyForceCmd := map[string]interface{}{
			"type":  "apply_force",
			"force": forceTotal,
		}
		forceBytes, _ := json.Marshal(applyForceCmd)
		_ = sendMessage(conn, string(forceBytes))

		fmt.Printf("[Cube %d] Pos: %.2f,%.2f,%.2f  Vel: %.2f,%.2f,%.2f  Force: %.2f,%.2f,%.2f\n",
			cubeID,
			pos[0], pos[1], pos[2],
			vel[0], vel[1], vel[2],
			forceTotal[0], forceTotal[1], forceTotal[2],
		)
		time.Sleep(100 * time.Millisecond)
	}

	fmt.Printf("[Cube %d] Control loop finished.\n", cubeID)
}

// ==============
// Main Function
// ==============

func main() {
	rand.Seed(time.Now().UnixNano())
	fmt.Println("Starting unsupervised agent with Paragon usage...")

	// 1) Run Paragon demo
	demoParagonUsage()

	// 2) Train or load the PCA model
	modelPath := "cube_pca_model.gob"
	nComponents := 2
	model, err := trainModelIfNeeded(modelPath, nComponents)
	if err != nil {
		fmt.Println("Failed to train or load model:", err)
		return
	}

	// 3) Control multiple cubes
	numCubes := 3           // Number of cubes to control
	controlDuration := 15.0 // Duration in seconds
	errorThreshold := 0.8   // Reconstruction error threshold
	var wg sync.WaitGroup

	for cubeID := 0; cubeID < numCubes; cubeID++ {
		wg.Add(1)
		go func(id int) {
			defer wg.Done()
			controlCube(model, id, controlDuration, errorThreshold)
		}(cubeID)
	}

	// Wait for all cubes to finish
	wg.Wait()
	fmt.Println("All cubes controlled successfully.")
}
