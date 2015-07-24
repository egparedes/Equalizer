
/* Copyright (c)      2015, Enrique G. Paredes <egparedes@ifi.uzh.ch>
 *               2008-2013, Stefan Eilemann <eile@equalizergraphics.com>
 *                    2010, Cedric Stalder <cedric.stalder@gmail.com>
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


#ifndef PLYLIB_ZBUFFERDIST_H
#define PLYLIB_ZBUFFERDIST_H

#include "api.h"
#include "typedefs.h"

#include <co/co.h>

namespace triply
{
/** Uses co::Object to distribute a model, holds a ZTreeBase node. */
class ZTreeDist : public co::Object
{
public:
    PLYLIB_API ZTreeDist();
    PLYLIB_API ZTreeDist( triply::ZTreeRoot* root );
    PLYLIB_API virtual ~ZTreeDist();

    PLYLIB_API void registerTree( co::LocalNodePtr node );
    PLYLIB_API void deregisterTree();

    PLYLIB_API triply::ZTreeRoot* loadModel( co::NodePtr master,
                                             co::LocalNodePtr localNode,
                                             const eq::uint128_t& modelID );
protected:
    PLYLIB_API ZTreeDist( ZTreeRoot* root,
                          ZTreeBase* node );

    unsigned getNumberOfChildren( ) const;

    PLYLIB_API virtual void getInstanceData( co::DataOStream& os );
    PLYLIB_API virtual void applyInstanceData( co::DataIStream& is );

private:
    ZTreeRoot* _root;
    ZTreeBase* _node;
    ZTreeDist* _childNodes[8];
    bool _isRoot;
};
}


#endif // PLYLIB_ZBUFFERDIST_H
