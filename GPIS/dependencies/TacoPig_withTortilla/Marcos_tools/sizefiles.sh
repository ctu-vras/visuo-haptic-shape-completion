#$ vi sizefiles.sh
# Bash to write the size of csv files in a directory
FILEIN=$1
FILEOUT=$2

#for FILE in $( ls $1 | grep .csv$) ; do   echo "Res: ${FILE}"; done
#o "Res: $( cat ${FILE} )" >> $2
#done
#for FILE in $( ls ./ | grep .csv$) ; do   echo "$(cat ${FILE} | csv-calc size)" >> temp.txt ; done
 #sed -n 's/.*sampled_//;s/.cs.*//p'
#for FILE in $(  ls -d -1 $PWD/$1/* | grep .csv$) ; do  echo " $(echo ${FILE} | sed -n 's/.*aser_//;s/_sampled.*//p')" "Res:" " $(echo ${FILE} | sed -n 's/.*sampled_//;s/.cs.*//p')" "#Points:" "$(cat ${FILE} | csv-calc size)" >> size_temp.txt; done
#for FILE in $(  ls -d -1 $PWD/$1/* | grep .csv$) ; do  echo " $1" "Res:" " $(echo ${FILE} | sed -n 's/.*sampled_//;s/.cs.*//p')" "#Points:" "$(cat ${FILE} | csv-calc size)" >> size_temp.txt; done
for FILE in $(  ls -d -1 $PWD/$1/* | grep .csv$) ; do  echo " $1" "Res:" " $(echo ${FILE} | sed -n 's/.*sampled_//;s/.cs.*//p')" "#Points:" "$(cat ${FILE} | csv-calc size | sed 's/,..*//g')" >> size_temp.txt; done

#Running all the instruction
#for FILE in $(ls -d */); do ./sizefiles.sh $(echo "${FILE}") ; done


#cat $1 | csv-calc size >> $2 
#printf '%s\n' 0a 50 . w | ed -s $2
#while read a;
#do echo $a
#   ./traveldist $a $1;
#done
 
