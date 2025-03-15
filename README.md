# Satellite Formation Control

This MATLAB script simulates and visualizes the motion of a satellite orbiting the Earth. The simulation uses Newtonian gravity, orbital dynamics, and the leapfrog integral method to calculate the satellite's orbit. It also uses consensus control to form the satellite into a star shape.
## Features

- **3D & 2D Visualization:** Displays two perspectives: a 3D view of the Earth and satellites and a 2D top-down view.
- **Orbital Dynamics:** Uses Newton's law of gravitation to compute satellite motion.
- **Leapfrog Integration:** Ensures stable numerical solutions for velocity and position updates.
- **Satellite Interaction Model:** Uses a Laplacian matrix to model satellite interactions.

## Requirements

- MATLAB (R2018b or later)
- `star_image_coordinate.mat` file containing star image data

## Usage

1. Load the script in MATLAB.
2. Ensure `star_image_coordinate.mat` is in the same directory.
3. Run the script to start the simulation.
4. A video (`satellite.avi`) is generated.

## Output

![Image](https://github.com/user-attachments/assets/b46a88f7-acd8-4157-868c-beda9f887e7c)

## License

This project is released under the MIT License.
