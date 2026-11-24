// Global btScalar alias (precision switch matching the library under test)
#if BT_USE_DOUBLE_PRECISION
global using btScalar = System.Double;
#else
global using btScalar = System.Single;
#endif
