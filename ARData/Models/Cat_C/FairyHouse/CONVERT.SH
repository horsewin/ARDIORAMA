for f in *
do
    mv $f `echo $f | tr '[a-z]' '[A-Z]'`
done