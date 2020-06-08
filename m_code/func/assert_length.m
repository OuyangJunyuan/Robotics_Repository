function yesorno = assert_length(input,rows2assert,cols2assert)

if (length(input(:,1))~= rows2assert) ||  (length(input(1,:))~= cols2assert)
    yesorno=0;
else
    yesorno=1;
end   
end

