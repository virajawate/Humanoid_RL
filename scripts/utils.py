import os
import re
import tempfile


def prepare_urdf(urdf_path, package_dir, package_name="g1_description"):
    """
    Prepare a ROS URDF for PyBullet by replacing package:// URIs
    with absolute filesystem paths.

    Args:
        urdf_path (str):
            Path to the original URDF file.

        package_dir (str):
            Absolute path to the ROS package directory.

        package_name (str):
            ROS package name used in package:// URIs.

    Returns:
        str:
            Path to the generated PyBullet-compatible URDF.
    """

    urdf_path = os.path.abspath(urdf_path)
    package_dir = os.path.abspath(package_dir)

    if not os.path.isfile(urdf_path):
        raise FileNotFoundError(
            f"URDF file not found: {urdf_path}"
        )

    if not os.path.isdir(package_dir):
        raise FileNotFoundError(
            f"Package directory not found: {package_dir}"
        )

    # Read original URDF
    with open(urdf_path, "r") as f:
        urdf = f.read()

    # Replace:
    # package://g1_description/meshes/xxx.stl
    #
    # with:
    # /absolute/path/g1_description/meshes/xxx.stl

    pattern = rf"package://{re.escape(package_name)}/"

    urdf = re.sub(
        pattern,
        package_dir.rstrip("/") + "/",
        urdf
    )

    # Create a temporary URDF for PyBullet
    temp_urdf = os.path.join(
        tempfile.gettempdir(),
        f"{package_name}_pybullet.urdf"
    )

    with open(temp_urdf, "w") as f:
        f.write(urdf)

    return temp_urdf
