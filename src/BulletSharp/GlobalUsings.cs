// In Bullet C++, the btScalar macro switches between float and double.
// Here the same effect is achieved with a global using + conditional compilation:
//   BulletSharp (float):         btScalar = System.Single
//   BulletSharp.Double (double): btScalar = System.Double (BT_USE_DOUBLE_PRECISION)
#if BT_USE_DOUBLE_PRECISION
global using btScalar = System.Double;
#else
global using btScalar = System.Single;
#endif
