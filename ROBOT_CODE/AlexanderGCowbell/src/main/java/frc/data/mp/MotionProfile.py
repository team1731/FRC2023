import sys
import csv
from os.path import isfile

lines = [
    "package frc.data.mp;\n",
    "\n",
    "public class MotionProfile {\n",
    "    public static ArmPath getArmPath() {\n",
    "        return new ArmPath(kNumPoints, proximalPoints, distalPoints, wristPoints);\n",
    "    }\n",
    "\n",
    "    public static final int kNumPoints = {numlines};\n",		
    "\n",
    "    // Position (ticks)	Velocity (RPM)\n",
    "\n",
	"    public static double [][]proximalPoints = new double[][]{\n",
	"    public static double [][]distalPoints = new double[][]{\n",
	"    public static double []wristPoints = new double[]{\n",
	"    };\n",
	"};\n"
]		

def header(fptr, numOfLines):
    for i in range(11):
        if i == 7:
            fptr.write(lines[7].format(numlines=numOfLines))
        else:
            fptr.write(lines[i])

def footer(fptr):
    fptr.write(lines[15])

def body(fptr, inlist, num, double):
    fptr.write(lines[num])
    items = len(inlist)

    for row in inlist[:-1]:
        if double:
            out = "        {{{}, {}}},\n".format(row[0], row[1])
        else:
            print(row)
            out = "        {},\n".format(row)
        fptr.write(out)

    row = inlist[-1]
    if double:
        out = "        {{{}, {}}}\n".format(row[0], row[1])
    else:
        out = "        {}\n".format(row)
    fptr.write(out)
    fptr.write(lines[14])
    fptr.write(lines[10])

def output(infile, outfile):

    with open(infile, newline='\n') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)
        inlist0 = []
        inlist1 = []
        inlist2 = []
        for row in reader:
            inlist0.append([row[0], row[2]])
            inlist1.append([row[1], row[3]])
            inlist2.append(row[4])

    with open(outfile+".java", "w") as f:
        header(f, len(inlist0))
        body(f, inlist0, 11, True)
        body(f, inlist1, 12, True)
        body(f, inlist2, 13, False)
        footer(f)

def main(infile, outfile):
    print('\nStarting\n')
    print('Writing file: {}\n'.format(outfile))
    output(infile, outfile)
    print('Ending\n')

if __name__ == '__main__':

    if len(sys.argv) >= 2:

        inputfilename = sys.argv[1]
        if isfile(inputfilename):
            outputfilename = ""
            if len(sys.argv) > 2:
                outputfilename = sys.argv[2]
            if outputfilename == "":
                outputfilename = "MotionProfile_" + inputfilename.split(".")[0]
            else:
                outputfilename = outputfilename.split(".")[0]

            print("Input  filename: {}".format(inputfilename))
            print("Output filename: {}".format(outputfilename))
            main(inputfilename, outputfilename)
        else:
            print("\n Input file does NOT exist!\n")

    else:
        print("\nYou must at least supply a csv inputfile !!")
        print("\nUsage: python3 MotionProfile.py <input filename> <output filename>\n")
