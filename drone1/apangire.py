def generate_square_kml(lat, lon, side_m=5, filename="square.kml"):
    start = (lat, lon)
    east = geodesic(meters=side_m).destination(start, 90)
    south_east = geodesic(meters=side_m).destination((east.latitude, east.longitude), 180)
    south = geodesic(meters=side_m).destination(start, 180)

    coords = [
        (lon, lat),
        (east.longitude, east.latitude),
        (south_east.longitude, south_east.latitude),
        (south.longitude, south.latitude),
        (lon, lat)
    ]

    kml = Kml()
    pol = kml.newpolygon(name="Square", outerboundaryis=coords)
    kml.save(filename)

    return filename


# --- PARSE POLYGON FROM KML ---
def parse_kml_polygon(kml_file):
    tree = ET.parse(kml_file)
    root = tree.getroot()
    ns = {"kml": "http://www.opengis.net/kml/2.2"}

    coords_text = root.find(".//kml:Polygon/kml:outerBoundaryIs/kml:LinearRing/kml:coordinates", ns).text.strip()
    coords = []

    for line in coords_text.split():
        lon, lat, *_ = line.split(",")
        coords.append((float(lat), float(lon)))

    return coords


# --- CONVERT POLYGON → WAYPOINT MISSION ---
def generate_waypoints(coords, altitude=2, hold_time=3):
    lines = ["QGC WPL 110"]
    seq = 0
    home_lat, home_lon = coords[0]

    lines.append(f"{seq}\t1\t0\t16\t0\t0\t0\t0\t{home_lat:.7f}\t{home_lon:.7f}\t{altitude}\t1"); seq+=1
    lines.append(f"{seq}\t0\t3\t22\t0\t0\t0\t0\t{home_lat:.7f}\t{home_lon:.7f}\t{altitude}\t1"); seq+=1

    for lat, lon in coords:
        lines.append(f"{seq}\t0\t3\t16\t{hold_time}\t0\t0\t0\t{lat:.7f}\t{lon:.7f}\t{altitude}\t1")
        seq += 1

    lines.append(f"{seq}\t0\t3\t21\t0\t0\t0\t0\t{home_lat:.7f}\t{home_lon:.7f}\t{altitude}\t1")

    return "\n".join(lines)
