---
layout: default
title: Home
nav_order: 1
description: "Kresling"
permalink: /
---


<html lang="en-US">
<head>
  <meta charset="UTF-8">
  <meta name="viewpoint" content="width=device-width, initial-scale=1.0">
  <!--<link rel="stylesheet" href="style.css"> -->
  <title><b>Kresling</b> Control </title>
</head>
<body>
  <div class="header-adder">
    <div class="title_set">
      <h1 style="text-align: center;"><strong>Kresling</strong> Control
      </h1>
    </div>
    <div class="names">
      <p style="text-align: center;"><strong><a href="http://www.kristendorsey.com/">Kristen L. Dorsey<sup>1*</sup></a>,<a href="https://nhansion.io">Nathaniel Hanson<sup>1</sup></a>,<a href="https://www.linkedin.com/in/immanuelampomah/"> Immanuel Ampomah Mensah<sup>1</sup></a>, Donelle Furline, Jr.<sup>1</sup>, Sonia F. Roberts<sup>2</sup>, <a href="https://www.linkedin.com/in/ashwu-0/">Celina Wu<sup>1</sup></a>, <a href="https://www.jesshealey.net/">Jessica Healey<sup>1</sup></a></strong></p>
      <p style="text-align: center;"><strong>PARSES Lab<sup>1</sup>, Electrical and Computer Engineering Department<sup>2</sup>
      <p style="text-align: center;"><strong>Northeastern University, Boston, MA, USA</strong></p>

  <div>
    <div style="position:relative;padding-top:0%;">
      <figure>
      <img src="images/test_seutp.jpg" alt="Kresling in test set up">
        <figcaption>
            Expanded Kresling set up in test setting.
        </figcaption>
  </figure>
    </div>
  </div>
  <h2>Abstract</h2>
  <p>As soft roboticists further investigate approaches
to position or force feedback control, accurate and embodied
proprioception (i.e., self-sensing) will become more important
to the control of soft robots in unstructured environments.
In this work, we demonstrate proprioceptive feedback control
of a soft, pneumatically-actuated origami robot. The robot
is a 41 mm long, 3-D printed Kresling-inspired structure
with six sets of sidewall folds and one degree of freedom.
Pneumatic actuation, provided by negative fluidic pressure,
causes the robot to contract. Capacitive sensors patterned onto
the robot provide position estimation and serve as input to
a feedback controller. Using a finite element approach, the
electrode shapes were optimized for sensitivity at larger (more
obtuse) fold angles to improve control across the actuation
range. We demonstrate stable position control through a series
of contraction set points from 2 mm to 16 mm, with error under
3 mm up to 10 mm contraction. Overall, this work contributes
optimized capacitive electrode design and the demonstration of
closed-loop feedback position control without visual tracking as
an input. This approach to capacitance sensing and modeling
constitutes a major step towards proprioceptive state estimation and feedback control in soft origami robotics.
  </p>
 
<div style="text-align: center;">
  <figure>
  
      <img src="images/system_architecture_mast_WHISPERS.png" alt="HYPER DRIVE System Architecture">
    <figcaption>
    Hyper-Drive system mounted to off-road mobile robot, with sample data representations of white reference target from a) the Visible to Near Infrared (VNIR) hyperspectral camera b) Shortwave Infrared hyperspectral camera c) High resolution RGB camera d) Combined point spectrometers.
    </figcaption>
  </figure>
</div>

  <p>
    <a href="https://github.com/PARSES-Lab/kresling_control/tree/main">Link to the Github</a>
  </p>
</body>
</html>
### [ArXiv Link Coming Soon!]()
### Bibtex
We hope this data benefits the integration of autonomy and hyperspectral imaging. If you use any of the data or methods from this project, please include this citation in your bibliography.
 ```
@misc{hanson2023hyperdrive,
  author = {Hanson, Nathaniel and Pyatski, Benjamin and Hibbard, Samuel and DiMarzio, Charles and Padır, Taşkın},
  keywords = {Robotics (cs.RO), Signal Processing (eess.SP), FOS: Computer and information sciences, FOS: Computer and information sciences, FOS: Electrical engineering, electronic engineering, information engineering},
  title = {Hyper-Drive: Visible-Short Wave Infrared Hyperspectral Imaging Data Sets for Robots in Unstructured Environments},
  year = {2023},
  copyright = {Creative Commons Attribution 4.0 International}
}

```
Correspondence: hanson [.] n [@] northeastern [.] edu


### Acknowledgements
Many thanks to the following collaborators for helping to organize and label this data:
* James Tukpah
* Austin Allison
* Rania Alshawabkeh
* Muneer Lalji


Funding Attribution:
```
Research was sponsored by the United States Army Core of Engineers (USACE) Engineer Research and Development Center (ERDC)
Geospatial Research Laboratory (GRL) and was accomplished under 
Cooperative Agreement Federal Award Identification Number (FAIN) W9132V-22-2-0001. 
The views and conclusions contained in this document are those of the authors 
and should not be interpreted as representing the official policies, either expressed or implied,
of USACE EDRC GRL or the U.S. Government. The U.S. Government is authorized to reproduce and distribute reprints for Government purposes notwithstanding any copyright notation herein.
```
