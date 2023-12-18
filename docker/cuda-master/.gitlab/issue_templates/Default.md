Hello!

Thank you for taking the time to report an issue with the CUDA container images.

**The CUDA container images are supported by the Kitmaker team at NVIDIA.** The Kitmaker
team is also responsible for the CUDA package repositories on
[developer.download.nvidia.com](http://developer.download.nvidia.com/compute/cuda/repos)
including the CUDA driver.

In order to ensure expedient support of your issue, first please read and understand the
following:

**1) Is your issue a problem with the nvidia-docker2 runtime? (A.K.A
     libnvidia-container)**

   This is usually the case if the issue can be tracked down to a problem with the driver
   libraries (or lack there of) being loaded into the container by the runtime.

   If this is the case, please search https://github.com/NVIDIA/libnvidia-container/issues
   for any issues that match the one being encountered and report there if not found.

   The NVIDIA container runtime and the CUDA container images are supported by separate teams.

**2) Is your issue caused by not being able to perform an update of the package system
     from within the container?**

   We do our best to make sure the packages we publish are available in all regions
   globally through our CDN provider. But sometimes our CDN has issues that are usually
   resolved within an hour or two after being encountered.

   If after more than six hours have passed and you have not been able to perform an
   update, then please check: https://github.com/NVIDIA/cuda-repo-management/issues

**3) Are you trying to pull a tag from Docker Hub or NGC and the container runtime is
     erroring out because the tag does not exist?**

   Starting in May 2023, we will begin deleting old unsupported tags from Docker Hub and NGC.

   This is being done to ensure that there are no un-patched critical vulnerabilities in
   these container image tags as we do not support them any longer.

   We strongly encourage anyone using expired tags to upgrade to newer supported container
   images. Supported tags are listed here:
   https://gitlab-master.nvidia.com/cuda-installer/cuda/-/blob/master/doc/supported-tags.md

   For those companies or individuals that do not want to upgrade to newer container
   images, we provide support in a build script provided in this same repository for
   self hosting older versions of the CUDA container images. See the project README for
   details.

   The CUDA container image support policy:
   https://gitlab-master.nvidia.com/cuda-installer/cuda/-/blob/master/doc/support-policy.md

   Thank you for your understanding as our small team tries to navigate a challenging
   container security environment.

**4) A tag for a cuDNN version is missing**

   cuDNN is developed by a team separate from the CUDA Toolkit. Sometimes on toolkit
   release day the cuDNN version for that toolkit is not yet ready so those tags will be
   missing. There might already be a ticket in the
   [CUDA Image Issue Tracker](https://gitlab.com/nvidia/container-images/cuda/-/issues)

   Other times, cuDNN does not release a version for a CUDA version, OS Distro, and
   architecture. Please check the [cuDNN download page](https://developer.nvidia.com/rdp/cudnn-download)
   to see if the expected CUDA version, OS Distro, and architecture download exists.

   Otherwise, please create a ticket! Sorry and thanks in advanced!

**If the issue is not any of the above, then please create a ticket WITHOUT THIS TEXT.**

Thank you!

********************

/label ~needs-triage

Describe your issue here!

