package main

import (
	"encoding/json"
	"fmt"
	"io"
	"net"
	"net/http"
	"strconv"
	"strings"
	"sync"
	"text/template"
	"time"
)
/* bind port for TCP socket
 * 'vision' data use raw image encode data protocol and relayTCPRaw function
 */
var TCPPortsDict = map[string]int{
	"message":10001,
}
// communication protocol for register information and 'message' data
type Message struct {
  Mtype string
  Pri int
  Id string
  Dest string
  Data string
}

var TCPIPs, TCPPorts, TCPSockets sync.Map
var typeDict sync.Map // true is server, false is client

var TCPSendCnt, TCPRelayCnt, TCPRecvCnt sync.Map

var TCPStateMap sync.Map
// Block the main function
var done = make(chan struct{})

// web visualization information
type SocketState struct {
	Key string
	Type string
	IP string
	Port int
	SendFPS int
	RelayFPS int
	RecvFPS int
}

// index.html resource
var myTemplate *template.Template
// data for web HTML
var socketStates []SocketState
// slice is not thread-safe
var mutex sync.RWMutex
// check error, function just prints the error information
func checkErr(err error){
	if err != nil {
		fmt.Println(err)
	}
}

// TCP step 1
func readTCP(socket *net.TCPListener, key string){
	for {
		conn, err := socket.Accept()
		checkErr(err)
		remoteAddr := conn.RemoteAddr()
		ip, _port, err := net.SplitHostPort(remoteAddr.String())
		port,err := strconv.Atoi(_port)
		checkErr(err)
		go handleTCP(conn, ip, port, remoteAddr, key)
	}
}
// TCP step 2
func handleTCP(conn net.Conn, ip string, port int, remoteAddr net.Addr, key string){
	defer conn.Close()
	buffer_data := make([]byte, 0)
	buffer_index := 0
	clear_flag := false
	for {
		if clear_flag {
			clear_flag = false
			buffer_data = make([]byte, 0)
			buffer_index = 0
		}
		data := make([]byte, 65535*5)
		n, err := conn.Read(data)
		if err == io.EOF || n == 0{
			conn.Close()
			conn = nil
			break
		}
		var message Message
		/*
		 * error type message
		 */
		//fmt.Println("11111111111111", buffer_index)
		if buffer_index > 65535 {
			clear_flag = true
			continue
		}
		if err := json.Unmarshal(data[:n], &message); err != nil {
			//go relayTCP(data, n, remoteAddr, key)
			// DO NOTHING
			//fmt.Println("json.Unmarshal err:", err)
			buffer_data = append(buffer_data, data[:n]...)
			buffer_index = buffer_index+n
			//fmt.Println("DEBUG:", data[:n])
			//fmt.Println("buffer_index:", buffer_index)
			if buffer_index == 43762 || buffer_index == 10996 {
				fmt.Println(buffer_index, "！！！！！")
				if err2 := json.Unmarshal(buffer_data[:buffer_index], &message); err2 != nil {
					fmt.Println("WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW", err2)
					fmt.Println("DEBUG:", buffer_data)
					buffer_data = make([]byte, 0)
					buffer_index = 0
					continue
				}
				//fmt.Println("333333333333333333333333")
				//buffer_data = make([]byte, 0)
				//buffer_index = 0
				clear_flag = true
			} else {
				continue
			}
		}
		checkErr(err)
		/*
		 * normal type message
		 */
		//fmt.Println("5555555555555555555")
		// register type message
		if message.Mtype == "register" {
			fmt.Printf("Register <%s:%s> as %s\n", key, message.Id, message.Data)
			// client or server ?
			if message.Data == "client"{
				typeDict.Store(message.Id, false)
			} else if message.Data == "server" {
				typeDict.Store(message.Id, true)
			} else {
				fmt.Println("Error register data", message.Data)
			}

			TCPIPs.Store(message.Id, ip)
			TCPPorts.Store(message.Id+":"+key, port)
			TCPSockets.Store(message.Id+":"+key, conn)
			feedback := Message{
				Mtype: "register",
				Pri: 5,
				Id: "000000",
				Dest: message.Id,
				Data: remoteAddr.String(),
			}
			feedbackStr, err := json.Marshal(feedback)
			checkErr(err)
			_, err = conn.Write(feedbackStr)
			checkErr(err)
			// state information
			id := message.Id
			isServer, ok := typeDict.Load(id)
			if !ok {
				fmt.Println("Error !!!!")
				continue
			}
			stateKey := "client:"+ id
			if isServer.(bool) {
				stateKey = "server:"+ id
			}
			SocketState := SocketState{Key:stateKey, Type:"TCP", IP: ip, Port: port, SendFPS:0, RelayFPS:0, RecvFPS:0}
			TCPStateMap.Store(message.Id+":"+key, SocketState)
			TCPSendCnt.Store(message.Id+":"+key, 0)
			TCPRelayCnt.Store(message.Id+":"+key, 0)
			TCPRecvCnt.Store(message.Id+":"+key, 0)
		} else { // other types message
			if clear_flag {
				go relayTCP(buffer_data, buffer_index, message, key)
			} else {
				go relayTCP(data, n, message, key)
			}
		}
		//fmt.Println("8888888888888888888")
	}
	// out of for, loose connection
	fmt.Printf("Close TCP connection <%s:%d> %s\n", ip, port, key)
	// try to delete key
	TCPIPs.Range(func(ID, otherIP interface{})bool{
		// same IP
		if ip == otherIP {
			// try to get port
			otherPort, ok := TCPPorts.Load(ID.(string)+":"+key)
			if ok {
				// same port
				if port == otherPort {
					TCPIPs.Delete(ID)
					TCPPorts.Delete(ID.(string)+":"+key)
					TCPSockets.Delete(ID.(string)+":"+key)
					typeDict.Delete(ID.(string))
					TCPSendCnt.Delete(ID.(string)+":"+key)
					TCPRelayCnt.Delete(ID.(string)+":"+key)
					TCPRecvCnt.Delete(ID.(string)+":"+key)
					return true
				}
			}
		}
		return true
	})
	fmt.Println("Finish TCP")
}

// TCP step 3
func relayTCP(data []byte, n int, message Message, key string) {
	fmt.Println("Relay data len:", n)
	// state info
	cnt, _ := TCPSendCnt.Load(message.Id+":"+key)
	// check type for safety
	switch cnt.(type) {
	case int:
		TCPSendCnt.Store(message.Id+":"+key, cnt.(int)+1)
	default:
		TCPSendCnt.Store(message.Id+":"+key, 0)
	}

	destID := message.Dest
	isServer, ok := typeDict.Load(message.Id)
	if !ok {
		fmt.Println("No ID in typeDict", message.Id)
		return
	}
	// server sends data to client directly
	if isServer.(bool) {
		socket, ok := TCPSockets.Load(destID+":"+key)
		if !ok {
			fmt.Println("No destination", destID+":"+key)
			//fmt.Println("Data: ", message.Data)
			return
		}
		// send
		socket.(net.Conn).Write(data[:n])
	} else { // find a best server for client
		destID = findServer()
		if destID == "nil" {
			fmt.Println("No server to send data")
			return
		}
		socket, ok := TCPSockets.Load(destID+":"+key)
		if !ok {
			fmt.Println("No destination", destID+":"+key)
			return
		}
		// send
		socket.(net.Conn).Write(data[:n])
	}

	// state info
	relayCnt, _ := TCPRelayCnt.Load(message.Id+":"+key)
	// check type for safety
	switch relayCnt.(type) {
	case int:
		TCPRelayCnt.Store(message.Id+":"+key, relayCnt.(int)+1)
	default:
		TCPRelayCnt.Store(message.Id+":"+key, 0)
	}

	recvCnt, _ := TCPRecvCnt.Load(destID+":"+key)
	// check type for safety
	switch recvCnt.(type) {
	case int:
		TCPRecvCnt.Store(destID+":"+key, recvCnt.(int)+1)
	default:
		TCPRecvCnt.Store(destID+":"+key, 0)
	}
}

func findServer() string{
	var serverId = "nil"
	typeDict.Range(func(id, isServer interface{})bool{
		if isServer.(bool) {
			serverId = id.(string)
		}
		return true
	})
	return serverId
}

func FPSCounter() {
	duration := time.Duration(time.Second)
	t := time.NewTicker(duration)
	defer t.Stop()
	for {
		<- t.C
		var states []SocketState
		TCPSockets.Range(func(idWithKey, socket interface{})bool{
			sendCnt, ok := TCPSendCnt.Load(idWithKey)
			if !ok {
				fmt.Println("No key in TCPSendCnt", idWithKey)
				return true
			}
			relayCnt, ok := TCPRelayCnt.Load(idWithKey)
			if !ok {
				fmt.Println("No key in TCPRelayCnt", idWithKey)
				return true
			}
			recvCnt, ok := TCPRecvCnt.Load(idWithKey)
			if !ok {
				fmt.Println("No key in TCPRecvCnt", idWithKey)
				return true
			}
			_state, ok := TCPStateMap.Load(idWithKey)
			state := _state.(SocketState)
			state.SendFPS = sendCnt.(int)
			state.RelayFPS = relayCnt.(int)
			state.RecvFPS = recvCnt.(int)
			states = append(states, state)

			id := strings.Split(idWithKey.(string), ":")[0]
			isServer, _ := typeDict.Load(id)
			key := "client:"+ id
			if isServer.(bool) {
				key = "server:"+ id
			}
			newState := SocketState{Key:key, Type:"TCP", IP: state.IP, Port: state.Port, SendFPS:0, RelayFPS:0, RecvFPS:0}
			TCPStateMap.Store(idWithKey, newState)
			TCPSendCnt.Store(idWithKey, 0)
			TCPRelayCnt.Store(idWithKey, 0)
			TCPRecvCnt.Store(idWithKey, 0)
			return true
		})
		mutex.Lock()
		socketStates = states
		mutex.Unlock()
	}
}

func initTemplate(fileName string) (err error) {
	myTemplate, err = template.ParseFiles(fileName)
	checkErr(err)
	return err
}

func webHandler(writer http.ResponseWriter, request *http.Request) {
	data := make(map[string]interface{})
	data["title"] = "Data Relay"
	mutex.RLock()
	data["states"] = socketStates
	mutex.RUnlock()
	myTemplate.Execute(writer, data)
}

func main() {
	for key, port := range TCPPortsDict {
		clientAddr, err := net.ResolveTCPAddr("tcp4", ":"+strconv.Itoa(port))
		checkErr(err)
		clientListener, err := net.ListenTCP("tcp", clientAddr)
		checkErr(err)
		go readTCP(clientListener, key)
	}
	go FPSCounter()
	initTemplate("./index.html")
	http.HandleFunc("/", webHandler)
	err := http.ListenAndServe("0.0.0.0:8080", nil)
	checkErr(err)
	<-done
}
