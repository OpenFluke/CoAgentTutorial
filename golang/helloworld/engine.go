package main

import (
	"fmt"

	"github.com/OpenFluke/PARAGON"
)

func main() {
	// Example: Create and train a simple network
	layerSizes := []struct{ Width, Height int }{
		{10, 1}, // Input layer
		{5, 1},  // Hidden layer
		{2, 1},  // Output layer
	}
	activations := []string{"linear", "relu", "softmax"}
	fullyConnected := []bool{true, true, true} // Adjusted to have 3 elements
	net := paragon.NewNetwork(layerSizes, activations, fullyConnected)

	// Dummy input and target data
	inputs := [][][]float64{{{1, 2, 3, 4, 5, 6, 7, 8, 9, 10}}}
	targets := [][][]float64{{{1, 0}}}
	net.Train(inputs, targets, 10, 0.01)

	fmt.Println("Training complete!")
}
