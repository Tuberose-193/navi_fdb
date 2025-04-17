# navi_fdb
 1.编译kissmatch
https://github.com/MIT-SPARK/KISS-Matcher
2.更改cmake中kissmatch库的位置 
 add_subdirectory(/home/auto/drivers/KISS-Matcher-main/cpp/kiss_matcher ${CMAKE_CURRENT_BINARY_DIR}/kiss_matcher)
3.更改yaml中配置的地图文件
记得检查地图是否在世界坐标系中正确
TODO: 地图初始位置添加，yaw可能修正不了，但pitch，roll，x,y,z可以直接给个初始值纠正变换在estimate之前
DONE: 合并globallocalization，添加配置文件（0406）
