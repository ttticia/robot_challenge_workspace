Refer to maze_examples

## General
Cells are 1.02m wide and square in shape

Walls are 53cm tall. some walls have gaps of 10-25cm.

## Terrain
Terrain tiles are raised by about 10cm.

types
1. 4 slopes in a concentric pattern with max height of 11cm
2. slats of 5cm thickness and 12cm tall with 17cm spacing
3. blobk rubble made of blocks multiple small bloacks up to 15cm tall

## Stairs
pyramidal shaped stairs with 3 levels. step height is 15cm and step width is 20cm. The stairs have railings on the side and steps are made of reflective metal.

## curtains
split curtains hang 0.9m above ground on edge of cell. robot should be able to pass through. split gap is 1-3cm. gap at the bottom of the curtains is about 15cm.

more example videos at: https://sutdapac-my.sharepoint.com/:f:/g/personal/tricia_ng_mymail_sutd_edu_sg/IgB4TfqDwsVbToo6ZaNN1FiBASF_wC2UGG8UhuoQ9ShRXwo?e=eK13DF

## scene understanding
maze will contain rooms that hold toy sets. these toy sets represent specific locations.

view maze_examples/rooms for examples of the toy sets.

toy sets will be placed in rooms (cell with 3 walls). The toyset will be placed at the back of the cell.

These rooms might contain yellow warning labels (see challenge booklet)

## survivors
posters of humans will be placed on the sides of rooms with toysets to represent survivors that need rescuing. some survivor posters might also have a 12x12cm QR code that gives hints to where another survivor might be.

## exit
the last cell will lead to a open area. this exit will be markes with a red line on the edge of the last cell leading to the open area. The robot can only see this line from the cell before the last cell.