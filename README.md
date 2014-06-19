# Prototype Device for the Detection of Subsurface Peach Pit Fragments
## Cal Poly Senior Project
### Pit Crew: Hamilton Little, Colby Lippincott, Rick Hayes, and Elliot Wenzel

========

#### Project Overview

The code in this repository was Developed for the Pit Crew Team Senior Project
at Cal Poly extending from Winter 2013 to Fall 2013. The entire project is open
source: The full project report is hosted by Cal Poly's Kennedy Library
(http://digitalcommons.calpoly.edu/mesp/197/), which details the machine's
mechanical design. The code base here is released under the Apache 2.0 license,
which enables nearly limitless reproduction, use of (for profit or not),
modification, distribution, etc. of the code found in this project (see notice
below). Code used in this project is often borrowed or based on the work of
others, who are credited at the bottom of this article.

========

#### Code Overview

This project was developed on an XMEGA-A1 XPLAINED development board. However,
the code would be applicable to any XMEGA series chip, with a minimal amount of porting
necessary.

Each folder in repository contains a test of a different subsystem. The folder
the reader should be  concerned with is the _system__testing_ folder; this 
contains the final version of the code used in the project. This is broken 
up into the following sections

- /src
   - High level application code and user interface (via virtual com port)
- /lib
   - Drivers devoped for the various peripheral devices
   - /atmel
      - Drivers provided by Atmel
- /inc
   - Shared data and macro definitions

The most useful of these likely the lib folder, which contains the drivers
developed for the following devices

- Sparkfun Devices
   - 16-channel multiplexer
   - 8-bit shift register
- Digikey Devices
   - AD7892 ADC
- Gecko Devices
   - G213V stepping motor driver

The stepping motor driver code, in particular, may be of use. It is a port of
the classic AVR446 (Linear Speed Control of a Stepper Motor) for the XMEGA
series chips.

========

#### Acknowledgments

1. Code used in this project draws from the example projects for the XMEGAA1
   XPLAIN board and the drivers for the XMEGA series, distributed by the Atmel
   Corporation. The author would like to thank Atmel for their work in developing
   and supporting the XMEGA chip series, as well as for the sample code, tutorials,
   and evaluation boards they provide for the XMEGA series.

2. Dr.John R. Ridgley, Cal Poly ME Professor, whose code developed for and
   instruction in the ME 405 (mechatronics) class at Cal Poly have enabled the
   author to advance this project in earnest.

Additionally, this project would not be possible without the contributions of
the following.

1. Wawona Frozen Foods, based in Clovis, California, for sponsoring the project.
   In particular, credit should be given to Bill Smittcamp, Brian Okland, Jose
   Valdez, Vincent Olliver, and Duncan Dowdle for their investment in the project,
   both monetary and personal.

2. Dr. Peter Schuster, Senior Project Advisor to the Pit Crew Team, whose
   inspiring direction and deep technical knowledge have enabled the Pit Crew to
   succeed in developin a novel concept, from requirement generation to prototype
   demonstration.
