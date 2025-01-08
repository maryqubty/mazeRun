# Maze Path Navigator with Drone and Car Guidance

This project demonstrates a multi-level system combining drone-based maze analysis and car navigation. Below is a detailed explanation of each phase, along with visual results from various mazes.

---

### **First Level: Path Analysis**
In this phase, the system extracts 3D point cloud data from COLMAP models and analyzes point depth to distinguish open paths from walls.

**Key Achievements:**
- Analyzed and processed 3D COLMAP models.
- Identified open paths and walls for any 3D maze.

📷
![image](https://github.com/user-attachments/assets/5fb9bf2e-e740-4a69-89ed-d23c52a65403) ![image](https://github.com/user-attachments/assets/fcc4588a-a0cf-40c0-886a-3b93f4cf9e5f)

![image](https://github.com/user-attachments/assets/6e476063-1b63-49b3-9220-a4e1c7f739e2) ![image](https://github.com/user-attachments/assets/07d4762b-fbfe-4bb5-8c67-1bd48b8cd5dc)

![image](https://github.com/user-attachments/assets/c4d3bb86-7739-4d48-aee9-ab1e43f98d1d) ![image](https://github.com/user-attachments/assets/9512b776-b3d0-408c-b849-75b213e26b75)

![image](https://github.com/user-attachments/assets/025c6099-e570-4221-8c14-06acbd4b9c9a) ![image](https://github.com/user-attachments/assets/1513f382-4572-4bcc-b557-3f9f1d25904c)



---

### **Second Level: 2D Grid Conversion and Pathfinding**
The second level focuses on converting 3D models into 2D grids and calculating the shortest path between manually selected start and end points.

**Key Features:**
- Created a 2D grid where open paths are represented by `0` and walls by `1`.
- Enabled user input for selecting start and exit points.
- Implemented the A* algorithm to compute static (non-real-time) shortest paths.

📷 *[Insert a picture of a 2D grid or a visualization of the pathfinding process here]*  
📷 *[Insert images showing maze grids with paths highlighted]*

---

### **Final Level: Real-Time Navigation**
The ultimate goal is to enable real-time communication between the drone and the car, optimizing the navigation process further.

**Planned Enhancements:**
- Enable real-time shortest-path directions from drone to car.
- Integrate path optimization algorithms for real-world implementation.

📷 *[Insert conceptual diagrams or planned visualizations for real-time navigation]*

---

### **Results from Different Mazes**
The system was tested on various maze models. Below are results showcasing the paths identified:

1. **Second Maze**  
📷 *[Insert a picture of the second maze and the identified path]*

2. **Third Maze**  
📷 *[Insert a picture of the third maze and the identified path]*

3. **Fourth Maze**  
📷 *[Insert a picture of the fourth maze and the identified path]*

---

### **Future Work**
- Real-time drone-to-car communication.
- Enhanced path optimization techniques.

Let me know if you'd like help filling in the details or formatting the visuals!
