第8周：Docker 安装与 ROS2 桌面容器 课堂笔记

 一、核心目标
用 Docker 一键运行 ROS2 桌面环境，通过浏览器访问图形界面，**解决环境不一致问题**，开箱即用。

 二、安装步骤

 Windows 安装 Docker
1.  下载：https://docs.docker.com/desktop/setup/install/windows-install/
2.  建议：安装包改名（无空格）；安装异常时，显示隐藏文件 → 删除 `ProgramData\DockerDesktop` 后重试
3.  安装：管理员权限打开 PowerShell → 输入 `& ` + 安装包路径并执行
4.  依赖：确保 WSL2 正常启用
 Mac 安装 Docker
 三、运行 ROS2 桌面容器
1.  使用项目：https://github.com/Tiryoh/docker-ros2-desktop-vnc
2.  拉取并启动镜像（按项目 Quick Start 第一条命令执行）
3.  测试小乌龟：

 四、为什么用 Docker
- ✅ 环境统一：不管 Windows/Mac/Linux，运行效果完全一致
- ✅ 干净隔离：不修改本机系统，用完即删
- ✅ 快速复现：实验环境一键部署，避免“在我电脑上是好的”
- ✅ 内置图形：浏览器直接用桌面，无需额外配置
