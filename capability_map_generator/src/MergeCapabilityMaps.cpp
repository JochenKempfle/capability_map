#include <tclap/CmdLine.h>
#include <string>
#include <vector>
#include <iostream>
#include "capability_map/CapabilityOcTree.h"

int main(int argc, char** argv)
{
    TCLAP::CmdLine cmd("Generates a capability map of the region specified by given bounding box.", ' ', "1.0");

    TCLAP::MultiArg<std::string> inputArg("i", "input", "The files to be merged\n\
                                             Example: -i mydir/filename1 -i myotherdir/filename2", true, "string");

    TCLAP::ValueArg<std::string> outputArg("o", "output", "filename and path where the merged capability map will be created.\n\
                                             Example: -o mydir/mysubdir/filename.cpm", true, "./capability_map.cpm", "string");
    cmd.add(inputArg);
    cmd.add(outputArg);

    // parse arguments with TCLAP
    try
    {
        cmd.parse(argc, argv);
    }
    catch (TCLAP::ArgException &e)  // catch any exceptions
    {
        printf("Error: %s for argument %s", e.error().c_str(), e.argId().c_str());
        exit(1);
    }

    std::vector<std::string> inputNames = inputArg.getValue();
    std::string outputName = outputArg.getValue();

    if (inputNames.size() < 2)
    {
        printf("Error: At least 2 input files must be given as arguments");
        exit(1);
    }

    std::vector<CapabilityOcTree*> trees;

    for (size_t i = 0; i < inputNames.size(); ++i)
    {
        trees.push_back(CapabilityOcTree::readFile(inputNames[i]));
    }

    // check if all trees are valid
    for (size_t i = 0; i < trees.size(); ++i)
    {
        if (trees[i] == NULL)
        {
            printf("Error: Capability map in file %s could not be loaded.\n", inputNames[i].c_str());
            exit(1);
        }
    }

    // check if all trees are valid and fit together
    for (size_t i = 1; i < trees.size(); ++i)
    {
        if (trees[0]->getResolution() != trees[i]->getResolution())
        {
            printf("Error: Capability map in file %s has resolution %g, capability map \
                    in file %s has resolution %g\n", inputNames[0].c_str(),
                    trees[0]->getResolution(), inputNames[i].c_str(), trees[i]->getResolution());
            exit(1);
        }else if (trees[0]->getGroupName() != trees[i]->getGroupName())
        {
            printf("Error: Capability map in file %s has group_name %s, capability map \
                    in file %s has group_name %s\n", inputNames[0].c_str(), trees[0]->getGroupName().c_str(),
                    inputNames[i].c_str(), trees[i]->getGroupName().c_str());
            exit(1);
        }
        else if (trees[0]->getBaseName() != trees[i]->getBaseName())
        {
            printf("Error: Capability map in file %s has base_name %s, capability map \
                    in file %s has base_name %s\n", inputNames[0].c_str(), trees[0]->getBaseName().c_str(),
                    inputNames[i].c_str(), trees[i]->getBaseName().c_str());
            exit(1);
        }
        else if (trees[0]->getTipName() != trees[i]->getTipName())
        {
            printf("Error: Capability map in file %s has tip_name %s, capability map \
                    in file %s has tip_name %s\n", inputNames[0].c_str(), trees[0]->getTipName().c_str(),
                    inputNames[i].c_str(), trees[i]->getTipName().c_str());
            exit(1);
        }
    }

    // loop through all trees
    for (size_t i = 1; i < trees.size(); ++i)
    {
        // loop through all tree nodes
        for(CapabilityOcTree::leaf_iterator it = trees[i]->begin_leafs(), end = trees[i]->end_leafs(); it != end; ++it)
        {
            if (trees[0]->getNodeCapability(it.getX(), it.getY(), it.getZ()).getType() == EMPTY)
            {
                trees[0]->setNodeCapability(it.getX(), it.getY(), it.getZ(),
                          trees[i]->getNodeCapability(it.getX(), it.getY(), it.getZ()));
            }
            else if (trees[0]->getNodeCapability(it.getX(), it.getY(), it.getZ()).getShapeFitError() >
                     trees[i]->getNodeCapability(it.getX(), it.getY(), it.getZ()).getShapeFitError())
            {
                if (trees[i]->getNodeCapability(it.getX(), it.getY(), it.getZ()).getType() != EMPTY)
                {
                    trees[0]->setNodeCapability(it.getX(), it.getY(), it.getZ(),
                              trees[i]->getNodeCapability(it.getX(), it.getY(), it.getZ()));
                }
            }
        }
    }

    if (!trees[0]->writeFile(outputName))
    {
        printf("Error: could not write to file %s.\n", outputName.c_str());
        exit(1);
    }
    else
    {
        printf("Capability map written to file %s.\n", outputName.c_str());
    }
}





