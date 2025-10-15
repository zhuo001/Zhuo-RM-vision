#!/usr/bin/env python3
"""
版本对比测试工具
比较 person_detect.py 和 person_detect_optimized.py 的性能差异
"""

import subprocess
import time
import sys
import os
from pathlib import Path

def run_version(script_name: str, duration: int = 30) -> dict:
    """
    运行指定版本并收集性能数据
    
    Args:
        script_name: 脚本文件名
        duration: 测试持续时间（秒）
    
    Returns:
        性能统计字典
    """
    print(f"\n{'='*60}")
    print(f"🔍 Testing: {script_name}")
    print(f"{'='*60}\n")
    
    script_path = Path(__file__).parent / script_name
    
    if not script_path.exists():
        print(f"❌ Script not found: {script_path}")
        return None
    
    print(f"⏱️  Running for {duration} seconds...")
    print("   Press Ctrl+C to stop early\n")
    
    start_time = time.time()
    
    try:
        # 启动进程
        process = subprocess.Popen(
            ['python3', str(script_path)],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1
        )
        
        # 等待指定时间或手动停止
        try:
            time.sleep(duration)
        except KeyboardInterrupt:
            print("\n⚠️  Interrupted by user")
        
        # 发送中断信号
        process.terminate()
        
        # 等待进程结束并获取输出
        stdout, stderr = process.communicate(timeout=5)
        
        elapsed = time.time() - start_time
        
        # 解析输出中的FPS信息
        fps_values = []
        for line in stdout.split('\n'):
            if 'FPS:' in line:
                try:
                    # 尝试提取FPS值
                    fps_str = line.split('FPS:')[1].split('|')[0].strip()
                    fps = float(fps_str)
                    fps_values.append(fps)
                except:
                    pass
        
        # 统计信息
        if fps_values:
            avg_fps = sum(fps_values) / len(fps_values)
            min_fps = min(fps_values)
            max_fps = max(fps_values)
        else:
            # 如果没有FPS信息，从统计中提取
            avg_fps = None
            min_fps = None
            max_fps = None
            
            for line in stdout.split('\n'):
                if 'Average FPS' in line or 'FPS' in line:
                    try:
                        fps_str = line.split(':')[-1].strip()
                        avg_fps = float(fps_str)
                    except:
                        pass
        
        return {
            'script': script_name,
            'elapsed_time': elapsed,
            'avg_fps': avg_fps,
            'min_fps': min_fps,
            'max_fps': max_fps,
            'stdout': stdout,
            'stderr': stderr,
            'return_code': process.returncode
        }
    
    except subprocess.TimeoutExpired:
        print("❌ Process timeout")
        process.kill()
        return None
    except Exception as e:
        print(f"❌ Error: {e}")
        return None

def print_comparison(results: list):
    """打印对比结果"""
    
    print("\n" + "="*80)
    print("📊 性能对比报告".center(80))
    print("="*80 + "\n")
    
    if not results or len(results) < 2:
        print("⚠️  需要至少两个有效的测试结果")
        return
    
    # 打印表头
    print(f"{'指标':<20} {'person_detect.py':<25} {'person_detect_optimized.py':<25} {'差异':>10}")
    print("-"*80)
    
    # 对比各项指标
    metrics = [
        ('平均FPS', 'avg_fps'),
        ('最小FPS', 'min_fps'),
        ('最大FPS', 'max_fps'),
        ('测试时长(s)', 'elapsed_time')
    ]
    
    for label, key in metrics:
        val1 = results[0].get(key)
        val2 = results[1].get(key)
        
        if val1 is not None and val2 is not None:
            if 'fps' in key.lower():
                diff_pct = ((val2 - val1) / val1) * 100
                print(f"{label:<20} {val1:>8.2f}              {val2:>8.2f}              {diff_pct:>+9.1f}%")
            else:
                print(f"{label:<20} {val1:>8.2f}              {val2:>8.2f}              N/A")
        else:
            print(f"{label:<20} {'N/A':<25} {'N/A':<25} {'N/A':>10}")
    
    print("\n" + "="*80)
    
    # 结论
    if results[0].get('avg_fps') and results[1].get('avg_fps'):
        improvement = ((results[1]['avg_fps'] - results[0]['avg_fps']) / results[0]['avg_fps']) * 100
        
        print("\n🎯 结论:")
        if improvement > 5:
            print(f"   ✅ 优化版本性能提升 {improvement:.1f}%")
        elif improvement < -5:
            print(f"   ⚠️  优化版本性能下降 {abs(improvement):.1f}%")
        else:
            print(f"   ℹ️  性能基本持平（差异 {improvement:.1f}%）")
        
        print(f"\n   原始版本: {results[0]['avg_fps']:.1f} FPS")
        print(f"   优化版本: {results[1]['avg_fps']:.1f} FPS")
    
    print("\n" + "="*80 + "\n")

def main():
    """主函数"""
    
    print("""
╔════════════════════════════════════════════════════════════════╗
║                    版本性能对比测试工具                          ║
╚════════════════════════════════════════════════════════════════╝

此工具将依次运行两个版本的检测程序，并对比性能差异。

测试配置:
  - 原始版本: person_detect.py
  - 优化版本: person_detect_optimized.py
  - 每个版本运行时长: 30秒（可手动中断）
  - 性能指标: FPS、稳定性

注意:
  - 请确保摄像头已连接
  - 测试过程中请勿移动摄像头或改变环境
  - 可随时按 Ctrl+C 跳过当前版本
""")
    
    input("按 Enter 开始测试...")
    
    # 测试两个版本
    results = []
    
    for script in ['person_detect.py', 'person_detect_optimized.py']:
        result = run_version(script, duration=30)
        if result:
            results.append(result)
            
            # 显示单个测试结果
            print(f"\n✓ 测试完成:")
            if result['avg_fps']:
                print(f"   平均FPS: {result['avg_fps']:.2f}")
                if result['min_fps'] and result['max_fps']:
                    print(f"   FPS范围: {result['min_fps']:.2f} - {result['max_fps']:.2f}")
            print(f"   返回码: {result['return_code']}")
        
        # 间隔
        if script == 'person_detect.py':
            print("\n⏸️  5秒后开始测试下一个版本...")
            time.sleep(5)
    
    # 打印对比报告
    if len(results) >= 2:
        print_comparison(results)
    else:
        print("\n❌ 测试失败，未能获取足够的数据进行对比")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠️  测试被用户中断")
        sys.exit(0)
    except Exception as e:
        print(f"\n❌ 发生错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
