
/* Copyright (c)      2015, Enrique G. Paredes <egparedes@ifi.uzh.ch>
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of Eyescale Software GmbH nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
*/


#ifndef TRIPLY_TREEGENERATOR_H
#define TRIPLY_TREEGENERATOR_H

#include "typedefs.h"
#include <algorithm>
#include <string>
#include <vector>

namespace triply
{

/*  The abstract base class for all kinds of tree generators.  */
class TreeGenerator
{
public:
    static const unsigned MaxProgressCount = 10;

    typedef TreeGenerator* ( *InstantiateFunctPtr )();

    // ---- TreeGenerator factory methods
    TRIPLY_API static TreeGenerator* instantiate( const std::string& name )
    {
        InstantiateFuncMap& lookupMap = getMapInstance();
        if( lookupMap.count( name ) > 0 )
        {
            return (*( lookupMap[ name ] )) ();
        }

        return 0;
    }

    TRIPLY_API static std::vector< std::string > getRegisteredNames()
    {
        std::vector<std::string> names;
        InstantiateFuncMap& lookupMap = getMapInstance();

        for( typename InstantiateFuncMap::const_iterator it=lookupMap.begin();
             it != lookupMap.end(); ++it )
        {
            names.push_back( it->first );
        }

        return names;
    }

    TRIPLY_API static bool isValidName( const std::string& name )
    {
        InstantiateFuncMap& lookupMap = getMapInstance();

        InstantiateFuncMap::const_iterator it=lookupMap.begin();
        while( it != lookupMap.end() && it->first != name )
            ++it;

        return it != lookupMap.end();
    }

    TRIPLY_API static std::string addGenerator( const std::string& name,
                                                InstantiateFunctPtr functPtr )
    {
        InstantiateFuncMap& lookupMap = getMapInstance();
        if( lookupMap.count( name ) > 0 && lookupMap[ name ] != functPtr )
        {
            TRIPLYASSERT( 0 );
            return "";
        }

        lookupMap[ name ] = functPtr;

        return name;
    }

    template <typename ActualGeneratorT>
    TRIPLY_API static std::string addGenerator(std::string name)
    {
        return addGenerator( name, &TreeGenerator::instantiate< ActualGeneratorT > );
    }

    // ---- TreeGenerator virtual interface
    TRIPLY_API virtual ~TreeGenerator() { }

    TRIPLY_API virtual const std::string getPartition() const = 0;

    TRIPLY_API virtual bool generate( VertexData& modelData,
                                      ModelTreeRoot& treeRoot,
                                      ModelTreeData& treeData,
                                      boost::progress_display& progress ) = 0;
                                      // progress bar => 10 steps

private:
    typedef std::map<std::string, InstantiateFunctPtr> InstantiateFuncMap;

    template < typename ActualGeneratorT >
    static TreeGenerator* instantiate()
    {
        return new ActualGeneratorT;
    }

    static InstantiateFuncMap& getMapInstance()
    {
        static InstantiateFuncMap instantiationMap_;
        return instantiationMap_;
    }
};

} // namespace

#endif // TRIPLY_TREEGENERATOR_H



