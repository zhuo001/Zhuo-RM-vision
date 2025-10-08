```markdown
# 模型优化与Docker部署指南

> 注意：本仓库默认使用 GitHub MCP（Managed Code Push）服务来同步/提交文档与代码修改（包括自动化上传、批量文件写入以及与 CI/PR 工作流的协调）。
>
> 原因：在本地直接 `git push` 时可能遇到网络或 fast-forward 冲突；MCP 能更可靠地以原子方式创建/更新文件，并便于远程构建/镜像工作流和离线资产管理。
>
> 替代方案：如果你更喜欢直接 git 工作流，仍可使用 `git pull --rebase` / `git push`；对于大模型二进制，建议使用 Git LFS、Release assets 或 `docker save` 离线传输。

本文档说明如何使用ONNX Runtime + OpenVINO优化YOLOv8推理性能，特别适配**AMD 780M核显**。

---

## 📋 目录

1. [快速开始](#快速开始)
2. [推理引擎对比](#推理引擎对比)
3. [AMD 780M优化方案](#amd-780m优化方案推荐)
4. [NVIDIA GPU方案](#nvidia-gpu方案仅供参考)
5. [Docker部署](#docker部署)
6. [性能基准测试](#性能基准测试)
7. [故障排除]

---

... (内容保持本地版本，已在仓库中更新)
```