README

This folder contains the code and data used to generate the results found in Sections 5 and 6 of the main text, and Section 2 of the Supplementary text.  These correspond to experiments which used an Intel RealSense camera that could provide RGB-D images as input, and the results that were generated using our "robust carving" algorithm, as opposed to the naive carving algorithm used to generate the results in Section 4 of the main text and Section 1 of the supplemental.

There are three driver scripts that were used to generate these results:

reconstruction_robust_carving.m:

Main driver script that implements imaging behind occluders algorithm. This script assumes that the user has access to background frames that can be used for shadow segmentation.  This script was used to generate results in Sections 5 and 6 of the main paper and is described in additional detail in Supplementary Section 2.2.

reconstruction_robust_carving_nobg.m:

Driver script that implements imaging behind occluders algorithm.  This implementation does not require background frames for shadow segmentation, and is described in Supplementary Section 2.3.

reconstruction_robust_carving_albedo:

Driver script that implements imaging behind occluders algorithm.  This implementation uses a shadow segmentation method that does not require background frames, and that can be applied when the visible relay surfaces have varying albedo.  The shadow segmentation method is described in the second half of Supplementary Section 2.3.

