close all
clear all
clc

fid = fopen('pyramid.STL','r');
out = fopen('pyramid_out.stl', 'w');
s = fgetl(fid); fprintf(out, '%s\n', s);

offset = [0.15748027/2*1000,0.08267717/2*1000,0.2*1000]';

for i=1:12
    s = fgetl(fid);     fprintf(out, '%s\n', s);  % facet normal
    s = fgetl(fid);     fprintf(out, '%s\n', s);  % outer loop
    for j=1:3
        %vstr = fscanf(fid, '%s');  % vertex
        vstr = fgetl(fid);
        vstr = strrep(vstr, 'vertex ', '');
        v = sscanf(vstr, '%f %f %f');
        v = (v - offset); %/ 1000;
        fprintf(out, '         vertex %f %f %f\n', v(1), v(2), v(3));
    end
    s = fgetl(fid);     fprintf(out, '%s\n', s);  % endloop
    s = fgetl(fid);     fprintf(out, '%s\n', s);  % endfacet
end


s = fgetl(fid); fprintf(out, '%s\n', s);
fclose(fid);
fclose(out);
