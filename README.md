# DiSLAM
Distributed SLAM Communication Template

### Installation
```
cd ~/
mkdir -p dislam_ws/src/
cd dislam_ws/src/
git clone https://github.com/ZJU-Robotics-Lab/DiSLAM.git
cd ..
catkin_make
```

### Run
```
cd src/DiSLAM/src/

# run broadcaster
python broadcaster.py

# run subscriber
python listener.py
```

### File Structure

* **src/broadcaster.py：**

Publish random DiSCO information: PointCloud2 LocalMap and Float32[] signature

* **src/listener.py：**

Subscribe DiSCO message from broadcaster
