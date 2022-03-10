整体架构：

robot -> informer

​                                 \

​                                   data-relay

​                                 /

sever <-  informer



因为robot/sever都需要收发消息，所以informer要有send函数以及recv线程

在config.py中配置相关信息

bind_port: just for test



注册后data-relay干的事情：

首先根据连接的ip和port（正常情况下一个ip只发一种信息，所以不需要port），记录id和对应的ip到TCPKnownList，如果是server则存到TCPServerList里；根据destination的id，记录ip和id，存到TCPDestDict



msg_send干的事：

自己看代码吧，写的很清楚



how to run：

```
$ cd src
$ . ./auto.sh
```

source(简写为.) 在当前shell运行，不需要执行权限

sh/bash 在subshell执行，不需要执行权限

./ 在subshell执行，需要执行权限



TODO：（知道要做但没空做）

web代码整合（现在都堆在server.go里，非常不优雅）

多通信？（忘了啥需求来着）

