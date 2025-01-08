# Maze Path Navigator with Drone and Car Guidance

This project demonstrates a multi-level system combining drone-based maze analysis and car navigation. Below is a detailed explanation of each phase, along with visual results from various mazes.

---

### **First Level: Path Analysis**
In this phase, the system extracts 3D point cloud data from COLMAP models and analyzes point depth to distinguish open paths from walls.

**Key Achievements:**
- Analyzed and processed 3D COLMAP models.
- Identified open paths and walls for any 3D maze.

<img src="https://github.com/user-attachments/assets/ba3fb419-1094-4829-813b-7ac79abd48f5
" width="150">
<img src="https://github.com/user-attachments/assets/0654a268-6422-49db-8928-08a79ea6b3ba
" width="150">
<img src="https://github.com/user-attachments/assets/01bcf63a-579a-4756-b8d7-dafe6149806c
" width="150">

<img src="https://github.com/user-attachments/assets/cf3f8861-ade2-45c2-b0bc-be505caf98df
" width="150">
<img src="https://github.com/user-attachments/assets/c3f3c824-148e-4337-8259-ad5a3516c616
" width="150">
<img src="https://github.com/user-attachments/assets/94976a82-c8b1-424a-b8b8-cdc40fc95f6a
" width="150">

<img src="https://github.com/user-attachments/assets/5268e53a-8756-4be1-b413-c24c382395c3
" width="150">
<img src="https://github.com/user-attachments/assets/483af13a-da37-4b71-83da-0d83b262ede6
" width="150">
<img src="https://github.com/user-attachments/assets/b35993c8-7096-484d-b7dc-659a5257c180
" width="150">

<img src="https://github.com/user-attachments/assets/dbb56e2b-e63e-4f33-9197-7d989529aa2f
" width="150">
<img src="https://github.com/user-attachments/assets/8b8e3986-3944-4c66-8623-fcb0e7b16c5e
" width="150">

---

### **Second Level: 2D Grid Conversion and Pathfinding**
The second level focuses on converting 3D models into 2D grids and calculating the shortest path between manually selected start and end points.

**Key Features:**
- Created a 2D grid where open paths are represented by `0` and walls by `1`.
- Enabled user input for selecting start and exit points.
- Implemented the A* algorithm to compute static (non-real-time) shortest paths.

<img src="https://github.com/user-attachments/assets/b8a3b6ec-1593-45a7-bf99-ce8c6d15b974
" width="150">
<img src="https://github.com/user-attachments/assets/e5cc3bd8-0707-466f-9fce-5b4009e95856
" width="200">

<img src="https://github.com/user-attachments/assets/122f2587-b992-4314-bb51-d7bddbd345af
" width="150">
<img src="https://github.com/user-attachments/assets/50df20be-86b8-48a9-9dee-2e215f77bdd3
" width="200">

<img src="https://github.com/user-attachments/assets/453a72ee-fd17-40df-8158-cc1f177c1d94
" width="150">
<img src="https://github.com/user-attachments/assets/70bd5e7e-9eb9-461c-93d4-bde1f0dc789f
" width="200">

<img src="https://github.com/user-attachments/assets/7a9b26fa-159f-4101-ae5f-f904d8d8956e
" width="150">
<img src="https://github.com/user-attachments/assets/16b82de3-0347-4434-b8c1-322faa75e512
" width="200">

---

### **Final Level: Real-Time Navigation**
The ultimate goal is to enable real-time communication between the drone and the car, optimizing the navigation process further.

**Planned Enhancements:**
- Enable real-time shortest-path directions from drone to car.

---

### **Future Work**
- Real-time drone-to-car communication.
