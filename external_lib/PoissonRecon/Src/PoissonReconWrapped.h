/*
Copyright (c) 2006, Michael Kazhdan and Matthew Bolitho
All rights reserved.

Modified by: Cedric Le Gentil (August 2024) for wrapping

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of
conditions and the following disclaimer. Redistributions in binary form must reproduce
the above copyright notice, this list of conditions and the following disclaimer
in the documentation and/or other materials provided with the distribution. 

Neither the name of the Johns Hopkins University nor the names of its contributors
may be used to endorse or promote products derived from this software without specific
prior written permission. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO THE IMPLIED WARRANTIES 
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
TO, PROCUREMENT OF SUBSTITUTE  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/

#ifndef POISSON_RECON_WRAPPED_INCLUDED
#define POISSON_RECON_WRAPPED_INCLUDED

// Remove gcc clang warning
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wdangling-else"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wmisleading-indentation"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wuninitialized"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wreorder"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic ignored "-Wclass-memaccess"
#pragma GCC diagnostic ignored "-Wdeprecated-copy"
#pragma GCC diagnostic ignored "-Wtype-limits"


#include "PreProcessor.h"
#include "Reconstructors.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include "MyMiscellany.h"
#include "PPolynomial.h"
#include "FEMTree.h"
#include "Ply.h"
#include "VertexFactory.h"
#include "DataStream.imp.h"

#include <eigen3/Eigen/Dense>


typedef float  Real;

class PoissonReconWrapped
{
	private:

		typename Reconstructor::Poisson::SolutionParameters< Real > sParams_;


		template< typename Real , unsigned int Dim , unsigned int FEMSig , bool HasGradients , bool HasDensity >
		std::pair<std::vector<Eigen::Matrix<Real,3,1> >, std::vector<Eigen::Matrix<int,3,1> > > getMesh
		(
			Reconstructor::Implicit< Real , Dim , FEMSig > &implicit ,
			const Reconstructor::LevelSetExtractionParameters &meParams
		);


		template< class Real , unsigned int Dim , unsigned int FEMSig , typename AuxDataFactory >
		std::pair<std::vector<Eigen::Matrix<Real,3,1> >, std::vector<Eigen::Matrix<int,3,1> > > execute();

		const std::vector< Eigen::Matrix<Real, 3,1> >& points_;
		const std::vector< Eigen::Matrix<Real, 3,1> >& normals_;

	public:
		PoissonReconWrapped(
			const std::vector< Eigen::Matrix<Real, 3,1> > &points
			,const std::vector< Eigen::Matrix<Real, 3,1> > &normals
			,unsigned int depth = 12
			,unsigned int adaptive_depth = 5
			,double scale = 1.1
			,double min_samples_per_node = 2.0
			,bool use_confidence = false
			,double interpolation_weight = 4.0
			,unsigned int gauss_seidel_iterations = 8
			);

		std::pair<std::vector<Eigen::Matrix<Real,3,1> >, std::vector<Eigen::Matrix<int,3,1> > > reconstruct();
};


#pragma GCC diagnostic pop

#endif // POISSON_RECON_WRAPPED_INCLUDED