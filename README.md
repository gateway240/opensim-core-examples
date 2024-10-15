# opensim-core-examples

## Minimal Bug Reproduction

### Triangle Inequality
For reproducing this error:
> what():  SimTK Exception thrown at MassProperties.h:542:
  Error detected by Simbody method Inertia::operator-=(): Diagonals of an Inertia matrix must satisfy the triangle inequality; got 0.000105365,0.00021073,0.00105365.
  (Required condition 'Ixx+Iyy+Slop>=Izz && Ixx+Izz+Slop>=Iyy && Iyy+Izz+Slop>=Ixx' was not met.)

 use `IMUIKTriangleInequality` example and build opensim-core. Tested with and without python or java bindings and it didn't make a difference

### Locale IO Problem
For reproducing this error:
>  what():  Timestamp at row 0 with value 0,000000 is greater-than/equal to timestamp at row 1 with value 0,000000
        Thrown at TimeSeriesTable.h:533 in validateRow().

 use the `IMUIKLocaleProblem` example 


## Build Instructions
1. Make sure you have opensim-core installed and on path


2. Change to example project directory

```
cmake -B build .
```
3. 
```
cd build
make
```