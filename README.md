# Box2D-Lite-CS
This is a C# 10 / .Net 6 port of the original Box2D-Lite C source code. I tried to keep the core of the code structure the same, though it isn't a perfect 1:1 translation. All of the raw pointers were translated into ref/span managed unsafe code.

This is designed as a resource, not as a ready-to-use library or nuget package.

Main changes:
* Body/Joint arrays are allocated in Main and passed as Spans to the World, instead of World keeping it's own lists.
* C# Refs cannot be stored on the heap like the C raw array element pointers, so BodyIndex was introduced to hold a Span element index, some of the World function signatures were modified to pass in the corresponding spans.
* C# does not support fixed arrays in structs that are not primitive types, so every Arbiter allocates it's set of 2 Contacts on the heap when created.
* OpenTK does not provide text rendering, so none of the text is printed in the demos. This is purely a visual thing, the hotkeys still work.
* Extra comments and links to a good video explanation of the Collision function were added.

## Box2D-Lite
Box2D-Lite is a small 2D physics engine. It was developed for the [2006 GDC Physics Tutorial](docs/GDC2006_Catto_Erin_PhysicsTutorial.pdf). This is the original version of the larger [Box2D](https://box2d.org) library. The Lite version is more suitable for learning about game physics.

## Source
* The original C source in this repo is from [box2d.org](https://web.archive.org/web/20151026045911/http://box2d.org/files/GDC2006/Box2D_Lite.zip).
* [erincatto/box2d-lite](https://github.com/erincatto/box2d-lite) is the official updated version.
* A great explanation of the Collision class is presented by [GamesWithGabe](https://www.youtube.com/channel/UCQP4qSCj1eHMHisDDR4iPzw) in his [Box2D Lite Walkthrough](https://youtu.be/0qqzGkbLbpM?list=PLtrSb4XxIVbpZpV65kk73OoUcIrBzoSiO) videos.