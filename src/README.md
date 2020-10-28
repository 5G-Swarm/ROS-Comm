# How to Use



* build DiSLAM in ROS work space:

  ```bash
  catkin_make --source src
  catkin_make install --source src
  ```

* load python lib:

  ```bash
  source devel/setup.bash
  ```

* run roscore

  ```bash
  roscore
  ```

* create a cloud server (http://cloud.tencent.com/) core number >= 4, memory >= 2G, bandwidth >= 8Mbps

* send local go-install-bag and go code to the cloud server

  ```bash
  local$ scp go1.15.3.linux-amd64.tar.gz username@<IP>:.
  local$ scp -r data_relay username@<IP>:.
  
  server$ sudo tar -C /usr/local -xzf go1.15.3.linux-amd64.tar.gz
  server$ export PATH=$PATH:/usr/local/go/bin
  ```

* build and run go server

  ```bash
  cd data_relay
  go build server.go
  ./server
  ```

* change IP address in python

  ```python
  from informer import Informer,config
  config.PUBLICT_IP = '42.192.10.208'
  ```

* run robot node and publish message

  ```bash
  python send2robot.py
  ```

* send message to the cloud server

  ```bash
  python robot.py
  ```

* receive message from the cloud server

  ```bash
  python server.py
  ```

* see GUI in web

  ```bash
  http://<IP-address>:8080
  ```

  

