"""
Guardian Mobility v0.0 - ROS Bag to 3DGS Reconstruction Service

企业级3D高斯溅射重建服务
基于ROS bag数据进行3D场景重建

技术栈:
- ROS bag 数据提取
- COLMAP 特征提取和SfM
- 3D Gaussian Splatting 训练
- 实时可视化

参考开源方案:
- graphdeco-inria/gaussian-splatting
- nerfstudio-project/nerfstudio
- colmap/colmap

@author Li Yixiang
@institution City University of Hong Kong
"""

import os
import sys
import subprocess
import json
import shutil
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, asdict
from datetime import datetime
import logging

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


@dataclass
class ReconstructionConfig:
    """重建配置"""
    # 输入配置
    rosbag_path: str
    output_dir: str

    # ROS配置
    image_topic: str = "/camera/image_raw"
    camera_info_topic: str = "/camera/camera_info"

    # COLMAP配置
    colmap_quality: str = "high"  # low, medium, high, extreme
    colmap_matcher: str = "exhaustive"  # exhaustive, sequential, vocab_tree

    # 3DGS配置
    iterations: int = 30000
    resolution: int = 1
    sh_degree: int = 3

    # 性能配置
    max_images: Optional[int] = None  # 限制图像数量
    downsample_factor: float = 1.0  # 图像下采样因子

    # GPU配置
    gpu_id: int = 0
    use_gpu: bool = True


@dataclass
class ReconstructionResult:
    """重建结果"""
    success: bool
    message: str
    output_path: Optional[str] = None
    colmap_path: Optional[str] = None
    gaussian_path: Optional[str] = None
    viewer_url: Optional[str] = None
    stats: Optional[Dict] = None
    error: Optional[str] = None


class ROSBagExtractor:
    """ROS Bag数据提取器"""

    def __init__(self, config: ReconstructionConfig):
        self.config = config
        self.output_images_dir = Path(config.output_dir) / "images"
        self.output_images_dir.mkdir(parents=True, exist_ok=True)

    def check_rosbag(self) -> bool:
        """检查ROS bag文件"""
        try:
            result = subprocess.run(
                ["rosbag", "info", self.config.rosbag_path],
                capture_output=True,
                text=True,
                timeout=30
            )
            return result.returncode == 0
        except Exception as e:
            logger.error(f"检查ROS bag失败: {e}")
            return False

    def extract_images(self) -> Tuple[bool, int]:
        """
        从ROS bag提取图像

        Returns:
            (success, image_count)
        """
        try:
            logger.info(f"开始从ROS bag提取图像: {self.config.rosbag_path}")

            # 使用Python脚本提取图像
            extract_script = self._create_extract_script()

            result = subprocess.run(
                ["python3", extract_script],
                capture_output=True,
                text=True,
                timeout=600
            )

            if result.returncode != 0:
                logger.error(f"图像提取失败: {result.stderr}")
                return False, 0

            # 统计提取的图像数量
            image_count = len(list(self.output_images_dir.glob("*.jpg")))
            logger.info(f"成功提取 {image_count} 张图像")

            return True, image_count

        except Exception as e:
            logger.error(f"提取图像异常: {e}")
            return False, 0

    def _create_extract_script(self) -> str:
        """创建图像提取脚本"""
        script_path = Path(self.config.output_dir) / "extract_images.py"

        script_content = f'''#!/usr/bin/env python3
import rosbag
import cv2
from cv_bridge import CvBridge
import os

bag_path = "{self.config.rosbag_path}"
image_topic = "{self.config.image_topic}"
output_dir = "{self.output_images_dir}"
max_images = {self.config.max_images or "None"}
downsample = {self.config.downsample_factor}

bridge = CvBridge()
bag = rosbag.Bag(bag_path, 'r')

count = 0
for topic, msg, t in bag.read_messages(topics=[image_topic]):
    if max_images and count >= max_images:
        break

    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 下采样
        if downsample != 1.0:
            h, w = cv_image.shape[:2]
            cv_image = cv2.resize(cv_image, (int(w*downsample), int(h*downsample)))

        # 保存图像
        filename = f"{{count:06d}}.jpg"
        cv2.imwrite(os.path.join(output_dir, filename), cv_image)
        count += 1

        if count % 10 == 0:
            print(f"已提取 {{count}} 张图像")
    except Exception as e:
        print(f"处理图像失败: {{e}}")
        continue

bag.close()
print(f"总共提取 {{count}} 张图像")
'''

        script_path.write_text(script_content)
        script_path.chmod(0o755)

        return str(script_path)


class COLMAPProcessor:
    """COLMAP处理器 - Structure from Motion"""

    def __init__(self, config: ReconstructionConfig):
        self.config = config
        self.images_dir = Path(config.output_dir) / "images"
        self.colmap_dir = Path(config.output_dir) / "colmap"
        self.colmap_dir.mkdir(parents=True, exist_ok=True)

    def run_colmap(self) -> bool:
        """运行COLMAP重建"""
        try:
            logger.info("开始COLMAP特征提取和匹配...")

            # 1. 特征提取
            if not self._feature_extraction():
                return False

            # 2. 特征匹配
            if not self._feature_matching():
                return False

            # 3. 稀疏重建
            if not self._sparse_reconstruction():
                return False

            # 4. 图像去畸变
            if not self._image_undistortion():
                return False

            logger.info("COLMAP重建完成")
            return True

        except Exception as e:
            logger.error(f"COLMAP处理异常: {e}")
            return False

    def _feature_extraction(self) -> bool:
        """特征提取"""
        database_path = self.colmap_dir / "database.db"

        cmd = [
            "colmap", "feature_extractor",
            "--database_path", str(database_path),
            "--image_path", str(self.images_dir),
            "--ImageReader.single_camera", "1",
            "--ImageReader.camera_model", "OPENCV",
            "--SiftExtraction.use_gpu", "1" if self.config.use_gpu else "0",
        ]

        result = subprocess.run(cmd, capture_output=True, text=True)
        return result.returncode == 0

    def _feature_matching(self) -> bool:
        """特征匹配"""
        database_path = self.colmap_dir / "database.db"

        cmd = [
            "colmap", f"{self.config.colmap_matcher}_matcher",
            "--database_path", str(database_path),
            "--SiftMatching.use_gpu", "1" if self.config.use_gpu else "0",
        ]

        result = subprocess.run(cmd, capture_output=True, text=True)
        return result.returncode == 0

    def _sparse_reconstruction(self) -> bool:
        """稀疏重建"""
        database_path = self.colmap_dir / "database.db"
        sparse_dir = self.colmap_dir / "sparse"
        sparse_dir.mkdir(exist_ok=True)

        cmd = [
            "colmap", "mapper",
            "--database_path", str(database_path),
            "--image_path", str(self.images_dir),
            "--output_path", str(sparse_dir),
        ]

        result = subprocess.run(cmd, capture_output=True, text=True)
        return result.returncode == 0

    def _image_undistortion(self) -> bool:
        """图像去畸变"""
        sparse_dir = self.colmap_dir / "sparse" / "0"
        dense_dir = self.colmap_dir / "dense"

        cmd = [
            "colmap", "image_undistorter",
            "--image_path", str(self.images_dir),
            "--input_path", str(sparse_dir),
            "--output_path", str(dense_dir),
            "--output_type", "COLMAP",
        ]

        result = subprocess.run(cmd, capture_output=True, text=True)
        return result.returncode == 0


class GaussianSplattingTrainer:
    """3D Gaussian Splatting训练器"""

    def __init__(self, config: ReconstructionConfig):
        self.config = config
        self.colmap_dir = Path(config.output_dir) / "colmap" / "dense"
        self.output_dir = Path(config.output_dir) / "gaussian_splatting"
        self.output_dir.mkdir(parents=True, exist_ok=True)

    def train(self) -> bool:
        """训练3DGS模型"""
        try:
            logger.info("开始3D Gaussian Splatting训练...")

            # 检查是否安装了gaussian-splatting
            if not self._check_gaussian_splatting():
                logger.error("未找到gaussian-splatting，请先安装")
                return False

            # 运行训练
            cmd = [
                "python3", "train.py",
                "-s", str(self.colmap_dir),
                "-m", str(self.output_dir),
                "--iterations", str(self.config.iterations),
                "--resolution", str(self.config.resolution),
                "--sh_degree", str(self.config.sh_degree),
            ]

            if self.config.use_gpu:
                env = os.environ.copy()
                env["CUDA_VISIBLE_DEVICES"] = str(self.config.gpu_id)
            else:
                env = None

            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                env=env,
                timeout=7200  # 2小时超时
            )

            if result.returncode != 0:
                logger.error(f"训练失败: {result.stderr}")
                return False

            logger.info("3DGS训练完成")
            return True

        except Exception as e:
            logger.error(f"训练异常: {e}")
            return False

    def _check_gaussian_splatting(self) -> bool:
        """检查gaussian-splatting是否安装"""
        try:
            result = subprocess.run(
                ["python3", "-c", "import gaussian_splatting"],
                capture_output=True
            )
            return result.returncode == 0
        except:
            return False


class ReconstructionPipeline:
    """重建流水线 - 主控制器"""

    def __init__(self, config: ReconstructionConfig):
        self.config = config
        self.extractor = ROSBagExtractor(config)
        self.colmap = COLMAPProcessor(config)
        self.gaussian = GaussianSplattingTrainer(config)

    def run(self) -> ReconstructionResult:
        """
        运行完整的重建流水线

        Returns:
            ReconstructionResult
        """
        start_time = datetime.now()

        try:
            # 1. 检查ROS bag
            logger.info("=" * 60)
            logger.info("Guardian Mobility - 3DGS重建流水线")
            logger.info("=" * 60)

            if not self.extractor.check_rosbag():
                return ReconstructionResult(
                    success=False,
                    message="ROS bag文件无效或不存在",
                    error="Invalid rosbag file"
                )

            # 2. 提取图像
            logger.info("\n[1/3] 提取图像...")
            success, image_count = self.extractor.extract_images()
            if not success or image_count == 0:
                return ReconstructionResult(
                    success=False,
                    message="图像提取失败",
                    error="Image extraction failed"
                )

            # 3. COLMAP重建
            logger.info(f"\n[2/3] COLMAP重建 ({image_count}张图像)...")
            if not self.colmap.run_colmap():
                return ReconstructionResult(
                    success=False,
                    message="COLMAP重建失败",
                    error="COLMAP reconstruction failed"
                )

            # 4. 3DGS训练
            logger.info("\n[3/3] 3D Gaussian Splatting训练...")
            if not self.gaussian.train():
                return ReconstructionResult(
                    success=False,
                    message="3DGS训练失败",
                    error="3DGS training failed"
                )

            # 计算统计信息
            end_time = datetime.now()
            duration = (end_time - start_time).total_seconds()

            stats = {
                "image_count": image_count,
                "duration_seconds": duration,
                "start_time": start_time.isoformat(),
                "end_time": end_time.isoformat(),
            }

            logger.info("\n" + "=" * 60)
            logger.info("重建完成！")
            logger.info(f"总耗时: {duration:.2f}秒")
            logger.info("=" * 60)

            return ReconstructionResult(
                success=True,
                message="重建成功",
                output_path=str(self.config.output_dir),
                colmap_path=str(self.colmap.colmap_dir),
                gaussian_path=str(self.gaussian.output_dir),
                viewer_url=f"http://localhost:8080/viewer?model={self.gaussian.output_dir}",
                stats=stats
            )

        except Exception as e:
            logger.error(f"重建流水线异常: {e}")
            return ReconstructionResult(
                success=False,
                message=f"重建失败: {str(e)}",
                error=str(e)
            )


def main():
    """命令行入口"""
    import argparse

    parser = argparse.ArgumentParser(description="ROS Bag to 3DGS Reconstruction")
    parser.add_argument("--rosbag", required=True, help="ROS bag文件路径")
    parser.add_argument("--output", required=True, help="输出目录")
    parser.add_argument("--image-topic", default="/camera/image_raw", help="图像话题")
    parser.add_argument("--iterations", type=int, default=30000, help="训练迭代次数")
    parser.add_argument("--max-images", type=int, help="最大图像数量")
    parser.add_argument("--gpu", type=int, default=0, help="GPU ID")

    args = parser.parse_args()

    # 创建配置
    config = ReconstructionConfig(
        rosbag_path=args.rosbag,
        output_dir=args.output,
        image_topic=args.image_topic,
        iterations=args.iterations,
        max_images=args.max_images,
        gpu_id=args.gpu
    )

    # 运行重建
    pipeline = ReconstructionPipeline(config)
    result = pipeline.run()

    # 输出结果
    print("\n" + "=" * 60)
    print("重建结果:")
    print(json.dumps(asdict(result), indent=2, ensure_ascii=False))
    print("=" * 60)

    sys.exit(0 if result.success else 1)


if __name__ == "__main__":
    main()
