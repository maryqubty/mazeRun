# Maze Path Navigator with Drone and Car Guidance

This project demonstrates a multi-level system combining drone-based maze analysis and car navigation. Below is a detailed explanation of each phase, along with visual results from various mazes.

---

### **First Level: Path Analysis**
In this phase, the system extracts 3D point cloud data from COLMAP models and analyzes point depth to distinguish open paths from walls.

**Key Achievements:**
- Analyzed and processed 3D COLMAP models.
- Identified open paths and walls for any 3D maze.

![2025-01-08](https://github.com/user-attachments/assets/304da0c6-8d80-41f2-aac9-085a51689981){:width="300"}
![image](https://github.com/user-attachments/assets/5fb9bf2e-e740-4a69-89ed-d23c52a65403){:width="300"}
![image](https://github.com/user-attachments/assets/6e476063-1b63-49b3-9220-a4e1c7f739e2){:width="300"}

![2025-01-08 (2)](https://github.com/user-attachments/assets/bdd39a23-d787-459b-9a80-bbfcfbaea9cc){:width="300"}
![image](https://github.com/user-attachments/assets/6e476063-1b63-49b3-9220-a4e1c7f739e2){:width="300"}
![image](https://github.com/user-attachments/assets/c4d3bb86-7739-4d48-aee9-ab1e43f98d1d){:width="300"}

![2025-01-08 (1)](https://github.com/user-attachments/assets/acf40005-d678-4e26-a225-9bf918b04952){:width="300"}
![image](https://github.com/user-attachments/assets/c4d3bb86-7739-4d48-aee9-ab1e43f98d1d){:width="300"}
![image](https://github.com/user-attachments/assets/1513f382-4572-4bcc-b557-3f9f1d25904c){:width="300"}

![image](https://github.com/user-attachments/assets/025c6099-e570-4221-8c14-06acbd4b9c9a){:width="300"}
![image](https://github.com/user-attachments/assets/1513f382-4572-4bcc-b557-3f9f1d25904c){:width="300"}

---

### **Second Level: 2D Grid Conversion and Pathfinding**
The second level focuses on converting 3D models into 2D grids and calculating the shortest path between manually selected start and end points.

**Key Features:**
- Created a 2D grid where open paths are represented by `0` and walls by `1`.
- Enabled user input for selecting start and exit points.
- Implemented the A* algorithm to compute static (non-real-time) shortest paths.

![image](https://github.com/user-attachments/assets/2e6bf3b6-454a-4756-a993-dd9e14954b49){:width="300"}
![image](https://github.com/user-attachments/assets/9dcc80c4-8430-476a-9653-44ea3a7975ee){:width="300"}

![image](https://github.com/user-attachments/assets/9e05d7f2-8a0f-4082-a8dd-996dbae56b76){:width="300"}
![image](https://github.com/user-attachments/assets/b06071ba-fa31-4cbb-bf78-2d81d2453847){:width="300"}

![image](https://github.com/user-attachments/assets/2a0c995f-e317-4668-8be4-733f554ebabc){:width="300"}
![image](https://github.com/user-attachments/assets/f3b31a55-3102-432c-b44e-6cbe23b3e5db){:width="300"}

![image](https://github.com/user-attachments/assets/68adcfaa-f1cc-43a0-b479-5bc3b0798f5e){:width="300"}
![image](https://github.com/user-attachments/assets/4ea8e294-adab-4216-8a88-a11f610af1e6){:width="300"}

---

### **Final Level: Real-Time Navigation**
The ultimate goal is to enable real-time communication between the drone and the car, optimizing the navigation process further.

**Planned Enhancements:**
- Enable real-time shortest-path directions from drone to car.

---

### **Future Work**
- Real-time drone-to-car communication.
