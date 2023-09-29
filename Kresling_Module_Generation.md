---
layout: default
title: Kresling Module Generation
has_children: false
---
# Kresling Module Generation

## Background
A Kresling module is a cylindrical origami shape that is representative of the buckling pattern formed when a cylinder of paper is [twisted and compressed](https://www.researchgate.net/publication/346643969_The_Fifth_Fold_Complex_Symmetries_in_Kresling-origami_Patterns). While the Kresling pattern originated as a paper structure, it can be 3D printed out of other compliant materials such as thermoplastic polyurethane (TPU). 3D printing enables Kresling modules to be fabricated with consistency and ease, and simplifies the process of adjusting its dimensions. 

[A script](../main/Kresling Generation/Kresling.py) run on Autodesk Fusion 360's Python API generates 3D Kresling models based on a set of adjustable user-input parameters. In this script, each fold of the module is drawn as two sets of lofted triangles. An explanation of the math is available in the [Kresling notes file](../main/Kresling Generation/Kresling_notes_for_CAD.pdf). An example STL generated using this script is available [here](../main/Kresling Generation/example_kresling.stl). 

## Prerequisites

In order to generate Kresling modules:

1. The [Kresling.py script](../main/Kresling Generation/Kresling.py) must be downloaded from the repository. Place the downloaded .py file into a folder titled "Kresling".
2. [Autodesk Fusion 360](https://www.autodesk.com/products/fusion-360/education) must be downloaded from the Autodesk site. Autodesk permits educational licenses for Fusion 360 to be used for university research.

## Script Usage Instructions

The script is run from Autodesk Fusion 360, using their built-in Python API. To load the script into Fusion 360:

1. Open Fusion 360 and select "UTILITIES" in the toolbar. 
2. Open the "ADD-INS" menu and select "Scripts and Add-Ins". 
3. Click the green "+" next to "My Scripts" in the "Scripts" tab. 
4. Navigate to the "Kresling" folder where the .py script lives.
5. Hit "Select Folder" to add the script to Fusion 360.
6. To run the script, select "Kresling" in the "My Scripts" list and click the "Run" button.
7. When the script runs, a new document opens in Fusion 360. A window with a set of adjustable parameters allows the user to modify critical dimensions on the Kresling. The Kresling model automatically updates while the parameters are altered.

![Input parameters can be adjusted and the resulting Kresling can be previewed.](images\script_parameters.png)

8. To generate the Kresling with the selected parameters, hit "OK".

## Export as STL

The Kresling model can be exported as an STL for 3D printing:

1. Open the "Bodies" dropdown on the feature tree.
2. The model is separated into different bodies so each part can be printed separately (such as the lid and the main Kresling structure). The main Kresling structure can be identified by its name, which lists the values of key parameters used to generate the model.

![The main body of the Kresling is named after the parameters used in its generation.](images\export_kresling_as_stl.png)

3. Right-click one of the bodies to export it as an STL. Select "Save As Mesh".
4. Ensure the "Format" is set to "STL (Binary)".
5. Hit "OK". Give the STL an appropriate file name and click "Save".