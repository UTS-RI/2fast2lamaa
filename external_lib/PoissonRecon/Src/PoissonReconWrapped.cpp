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

#include "PoissonReconWrapped.h"

template< typename Real , unsigned int Dim , unsigned int FEMSig , bool HasGradients , bool HasDensity >
std::pair<std::vector<Eigen::Matrix<Real,3,1> >, std::vector<Eigen::Matrix<int,3,1> > > PoissonReconWrapped::getMesh
(
	Reconstructor::Implicit< Real , Dim , FEMSig > &implicit ,
	const Reconstructor::LevelSetExtractionParameters &meParams
)
{
	// A description of the output vertex information
	using VInfo = Reconstructor::OutputVertexInfo< Real , Dim , HasGradients , HasDensity >;

	// A factory generating the output vertices
	using Factory = typename VInfo::Factory;
	Factory factory = VInfo::GetFactory();


	// A backing stream for the vertices
	Reconstructor::OutputInputFactoryTypeStream< Factory > vertexStream( factory , true , false );
	Reconstructor::OutputInputFaceStream< Dim-1 > faceStream( true , true );

	{
		// The wrapper converting native to output types
		typename VInfo::StreamWrapper _vertexStream( vertexStream , factory() );

		// Extract the level set
		implicit.extractLevelSet( _vertexStream , faceStream , meParams );
	}

	vertexStream.reset();
	std::vector< Eigen::Matrix<Real,3,1> > vertices;
	size_t vertexCount = vertexStream.size();
	vertices.reserve( vertexCount );
	for( size_t i=0; i<vertexCount ; i++ )
	{
		typename Factory::VertexType vertex = factory();
		if( !vertexStream.read( vertex ) ) ERROR_OUT( "Failed to read vertex " , i , " / " , 10 );
		vertices.push_back( Eigen::Matrix<Real,3,1>( vertex[0] , vertex[1] , vertex[2] ) );
	}

	faceStream.reset();
	std::vector< Eigen::Matrix<int,3,1> > faces;
	size_t faceCount = faceStream.size();
	faces.reserve( faceCount );
	// write faces
	std::vector< int > polygon;
	//ply->put_element_setup( "face" );
	for( size_t i=0 ; i<faceCount ; i++ )
	{
		PLY::Face< int > ply_face;
		if( !faceStream.read( polygon ) ) ERROR_OUT( "Failed to read polygon " , i , " / " , 10 ); 
		ply_face.nr_vertices = int( polygon.size() );
		ply_face.vertices = new int[ polygon.size() ];
		for( int j=0 ; j<int(polygon.size()) ; j++ ) ply_face.vertices[j] = (int)polygon[j];
		faces.push_back( Eigen::Matrix<int,3,1>( ply_face.vertices[0] , ply_face.vertices[1] , ply_face.vertices[2] ) );
		delete[] ply_face.vertices;
	}

	return std::make_pair( vertices , faces );
}


template< class Real , unsigned int Dim , unsigned int FEMSig , typename AuxDataFactory >
std::pair< std::vector<Eigen::Matrix<Real,3,1> >, std::vector<Eigen::Matrix<int,3,1> > > PoissonReconWrapped::execute()
{
	static const bool HasAuxData = !std::is_same< AuxDataFactory , VertexFactory::EmptyFactory< Real > >::value;

	///////////////
	// Types --> //
	using namespace VertexFactory;

	// The factory for constructing an input sample's data
	typedef typename std::conditional< HasAuxData , Factory< Real , NormalFactory< Real , Dim > , AuxDataFactory > , NormalFactory< Real , Dim > >::type InputSampleDataFactory;

	// The factory for constructing an input sample
	typedef Factory< Real , PositionFactory< Real , Dim > , InputSampleDataFactory >  InputSampleFactory;

	typedef InputDataStream< typename InputSampleFactory::VertexType > InputPointStream;

	// The type storing the reconstruction solution (depending on whether auxiliary data is provided or not)
	using Implicit = typename std::conditional< HasAuxData , Reconstructor::Poisson::Implicit< Real , Dim , FEMSig , typename AuxDataFactory::VertexType > , Reconstructor::Poisson::Implicit< Real , Dim , FEMSig > >::type;
	// <-- Types //
	///////////////


	Implicit *implicit = NULL;
	Reconstructor::LevelSetExtractionParameters meParams;


	meParams.linearFit = false;
	meParams.outputGradients = false;
	meParams.forceManifold = true;
	meParams.polygonMesh = false;
	meParams.verbose = false;


	InputSampleFactory *_inputSampleFactory;
	_inputSampleFactory = new InputSampleFactory( VertexFactory::PositionFactory< Real , Dim >() , VertexFactory::NormalFactory< Real , Dim >() );
	InputSampleFactory &inputSampleFactory = *_inputSampleFactory;

	std::vector< typename InputSampleFactory::VertexType > inCorePoints;
	InputPointStream *pointStream;

	// Get the point stream
	{
		pointStream = new    PLYInputDataStream< InputSampleFactory >( "" , inputSampleFactory );
	}

	typename Reconstructor::Poisson::EnvelopeMesh< Real , Dim > *envelopeMesh = NULL;

	// A wrapper class to realize InputDataStream< SampleType > as an InputSampleStream
	struct _InputSampleStream final : public Reconstructor::InputSampleStream< Real , Dim >
	{
		typedef Reconstructor::Normal< Real , Dim > DataType;
		typedef VectorTypeUnion< Real , Reconstructor::Position< Real , Dim > , DataType > SampleType;
		typedef InputDataStream< SampleType > _InputPointStream;
		_InputPointStream &pointStream;
		size_t counter = 0;
		const std::vector<Eigen::Matrix<Real,3,1> > &points;
		const std::vector<Eigen::Matrix<Real,3,1> > &normals;

		_InputSampleStream( 
			_InputPointStream &pointStream
			, const std::vector<Eigen::Matrix<Real,3,1> > &points
			, const std::vector<Eigen::Matrix<Real,3,1> > &normals
			) : pointStream( pointStream )
			, points(points)
			, normals(normals)
		{
		}
		void reset( void ){ pointStream.reset();  counter = 0; }
		bool base_read( Reconstructor::Position< Real , Dim > &p , Reconstructor::Normal< Real , Dim > &n ) 
		{
			if (counter == points.size())
				return false;
			p.coords[0] = points[counter][0];
			p.coords[1] = points[counter][1];
			p.coords[2] = points[counter][2];

			n.coords[0] = normals[counter][0];
			n.coords[1] = normals[counter][1];
			n.coords[2] = normals[counter][2];

			counter++;
			return true;
		}
	};


	_InputSampleStream sampleStream( *pointStream , points_, normals_ );

	implicit = new typename Reconstructor::Poisson::Implicit< Real , Dim , FEMSig >( sampleStream , sParams_ , envelopeMesh );

	delete pointStream;
	delete _inputSampleFactory;
	delete envelopeMesh;


	auto mesh = getMesh< Real , Dim , FEMSig ,false , false >(*implicit , meParams );

	delete implicit;

	return mesh;
}



PoissonReconWrapped::PoissonReconWrapped(
	const std::vector< Eigen::Matrix<Real,3,1> > &points
	,const std::vector< Eigen::Matrix<Real,3,1> > &normals
	,unsigned int depth
	,unsigned int adaptive_depth
	,double scale
	,double min_samples_per_node
	,double interpolation_weight
	,unsigned int gauss_seidel_iterations
	):
		points_(points),
		normals_(normals)

{
	// Parameters to change in the future
	sParams_.iters = (unsigned int)gauss_seidel_iterations;
	sParams_.scale = (Real)scale;
	sParams_.pointWeight = (Real)interpolation_weight;
	sParams_.samplesPerNode = (Real)min_samples_per_node;
	sParams_.depth = (unsigned int)depth;
	sParams_.fullDepth = (unsigned int)adaptive_depth;

	sParams_.verbose = false;
	sParams_.dirichletErode = true;
	sParams_.outputDensity = false;
	sParams_.exactInterpolation = false;
	sParams_.showResidual = false;
	sParams_.confidence = (Real)0.0;
	sParams_.confidenceBias = (Real)0.0;
	sParams_.lowDepthCutOff = (Real)0.0;
	sParams_.width = (Real)0.0;
	sParams_.cgSolverAccuracy = (Real)0.001;
	sParams_.targetValue = (Real)0.5;
	sParams_.baseDepth = (unsigned int)-1;
	sParams_.solveDepth = (unsigned int)-1;
	sParams_.kernelDepth = (unsigned int)-1;
	sParams_.envelopeDepth = (unsigned int)-1;
	sParams_.baseVCycles = (unsigned int)1;
	sParams_.alignDir = 2;
}

std::pair<std::vector<Eigen::Matrix<Real,3,1> >, std::vector<Eigen::Matrix<int,3,1> > > PoissonReconWrapped::reconstruct()
{
	Timer timer;
	ThreadPool::Init( (ThreadPool::ParallelType)ThreadPool::THREAD_POOL , std::thread::hardware_concurrency() );


	auto mesh = execute<Real, 3, FEMDegreeAndBType<2,BOUNDARY_NEUMANN>::Signature, VertexFactory::EmptyFactory< Real >>();

	ThreadPool::Terminate();
	return mesh;
}
