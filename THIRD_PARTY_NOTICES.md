# Third-Party Notices

The MIT License in this repository applies only to the original project code.
Third-party software and assets retain their own copyright and license terms.

## Slamtec RPLIDAR SDK

- Upstream: <https://github.com/Slamtec/rplidar_sdk>
- Integration: Git submodule at `src/third_party/rplidar_sdk`
- Pinned revision: `99478e5fb90de3b4a6db0080acacd373f8b36869`
- Upstream license: BSD 2-Clause

The SDK's complete license text remains in the submodule's `LICENSE` file.

## Waveshare RoArm ROS 2 workspace

- Upstream: <https://github.com/waveshareteam/roarm_ws>
- Integration: fetched during the ROS 2 container build and CI; it is not
  vendored in this repository
- Pinned revision: `40dbd84b553695212fab713e8465f817ba95454d`

The upstream repository does not publish one repository-wide license at that
revision. Individual ROS packages declare their own terms, including MIT and
BSD, while some package manifests contain an unresolved license placeholder.
Review the license declarations and source-file notices for the specific
packages used before redistributing a container image or derived vendor code.

## Other dependencies

Python packages listed in `requirements.txt`, ROS 2 / MoveIt 2 packages, base
container images, and system packages are installed as external dependencies.
They are not relicensed by this project; their upstream terms continue to
apply.
