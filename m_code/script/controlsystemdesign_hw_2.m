clear all;
clc;

y1=ppt_mothed(16);
y2=textbook_mothed(16);
dy=y1-y2;
plot(1:length(dy),dy);