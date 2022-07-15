function pwr_kml(name,latlon)
%makes a kml file for use in google earth
%input:  name of track, one matrix containing latitude and longitude
%usage:  pwr_kml('track5',latlon)

% header=['<kml xmlns="http://earth.google.com/kml/2.2"><Placemark><description>"' name '"</description><LineString><tessellate>1</tessellate><coordinates>'];
header=['<?xml version="1.0" encoding="UTF-8"?><kml xmlns="http://www.opengis.net/kml/2.2" xmlns:gx="http://www.google.com/kml/ext/2.2" xmlns:kml="http://www.opengis.net/kml/2.2" xmlns:atom="http://www.w3.org/2005/Atom"><Placemark><description>"Track_190704"</description><LineString><tessellate>1</tessellate><coordinates> '];
footer='</coordinates></LineString></Placemark></kml>';

fid = fopen([name '.kml'], 'wt');
d=flipud(rot90(fliplr(latlon)));
% fprintf(fid, 'long,lat\n');
fprintf(fid, '%s \n',header);
fprintf(fid, '%.15f,%.15f,0.0\n', d);
% fprintf(fid, '%.10f,%.10f\n', d);
fprintf(fid, '%s', footer);
fclose(fid);