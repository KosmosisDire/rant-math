/* Places on the Earth as GeoPoint holds them: latitude and longitude in degrees on the WGS84
 * ellipsoid, and altitude in meters above that ellipsoid, as GNSS gives them. */
#ifndef RANT_MATH_GEO_H
#define RANT_MATH_GEO_H

#include "base.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Meters from the Earth's center, x toward latitude 0 and longitude 0, z toward the north pole.
 * Back, longitude is in (-180, 180] and a pole's is 0. */
RANT_MATH_API RantDouble3  rant_geo_point_to_ecef(RantGeoPoint point);
RANT_MATH_API RantGeoPoint rant_geo_point_from_ecef(RantDouble3 ecef);
/* point in meters east, north and up of origin, the flat frame a robot maps a site in. */
RANT_MATH_API RantDouble3  rant_geo_point_to_enu(RantGeoPoint origin, RantGeoPoint point);
RANT_MATH_API RantGeoPoint rant_geo_point_from_enu(RantGeoPoint origin, RantDouble3 enu);

#ifdef __cplusplus
}
#endif
#endif /* RANT_MATH_GEO_H */
