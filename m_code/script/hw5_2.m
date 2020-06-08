Rz=Rort([0,0,1]',d2r(-20))
Ry=Rort([0,1,0]',d2r(-110))
R=Rz*Ry
gbc=gab_Rp(R,[7,-2,5]')
qc=[0.5,0.2,3.2,1]'
qb=gbc*qc