# dropletCA

- (1) "grabcutPlusV3.py": Initial commit of this script contains an un-formatted, non-vectorized algorithm for the GrabCut Image Segmentation + Rotated Ellipse Fitting implementation.
- (2) "grabcutPlusV3-copy.py": Initial commit of this script contains an un-formatted, non-vectorized algorithm where "s" lets you see results for ellipse fitting after thresholding and muting all pixels outside the bounding box "mask" (without running GrabCut).
- (3) "Images": contains lateral view droplet images used during development.
- (4) "Droplet Contact Angle Tool Summary.docx": a word doc that has an explanation of the implementation for (1).

Potential Future Updates: 
- (1,2): Format (lint) file
- (1,2): Vectorize code (remove inefficient variables, loops, etc.) and make it modular.
- (1,2): Get code ready for integration with Mask R-CNN or other model.
- (3): Add a references file to track what camera was used for capturing etc.
