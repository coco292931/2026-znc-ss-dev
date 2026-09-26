# rewrite 独立开发目录

本目录从 2026-znc-ss 提取，保留 rewrite 目标程序的编译输入、必要头文件、Loongson 驱动、部署/运行脚本和运行时模型。

编译入口：rewrite/build_rewrite.sh target
部署入口：deploy_rewrite.ps1 -> rewrite/deploy_rewrite.sh
GUI：tools/start_rewrite_gui.bat

未复制旧版巡线、expt、测试/回放示例和历史资料。
