/*
Copyright (c) 2006, Michael Kazhdan and Matthew Bolitho
All rights reserved.

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

#ifndef REGULAR_TREE_NODE_INCLUDED
#define REGULAR_TREE_NODE_INCLUDED

#include <functional>
#include "Streams.h"
#include "Allocator.h"
#include "BinaryNode.h"
#include "Window.h"

template< unsigned int Dim , class NodeData , class DepthAndOffsetType >
struct RegularTreeNode
{
	// This struct temporarily makes a node appear to be a root node by removing the parent reference
	struct SubTreeExtractor
	{
		SubTreeExtractor( RegularTreeNode &root ) : _root(root)
		{
			_rootParent = _root.parent;
			_root.parent = NULL;
			_root.depthAndOffset( _depth , _offset );
			int depth=0 , offset[Dim];
			for( unsigned int d=0 ; d<Dim ; d++ ) offset[d] = 0;
			RegularTreeNode::ResetDepthAndOffset( &_root , depth , offset );
		}
		SubTreeExtractor( RegularTreeNode *root ) : SubTreeExtractor( *root ){}
		~SubTreeExtractor( void )
		{
			RegularTreeNode::ResetDepthAndOffset( &_root , _depth , _offset );
			_root.parent = _rootParent;
		}
	protected:
		RegularTreeNode &_root , *_rootParent;
		int _depth , _offset[Dim];
	};
	struct DepthAndOffset
	{
		DepthAndOffsetType depth , offset[Dim];
		DepthAndOffset( void ){ depth = 0 , memset( offset , 0 , sizeof(DepthAndOffsetType) * Dim ); }
		DepthAndOffset( DepthAndOffsetType d , const DepthAndOffsetType off[] ){ depth = d , memcpy( offset , off , sizeof(DepthAndOffsetType) * Dim ); }
		bool operator == ( const DepthAndOffset &doff ) const
		{
			if( depth!=doff.depth ) return false;
			for( int d=0 ; d<Dim ; d++ ) if( offset[d]!=doff.offset[d] ) return false;
			return true;
		}

		friend std::ostream &operator << ( std::ostream &os , const DepthAndOffset &depthAndOffset )
		{
			os << "( " << depthAndOffset.offset[0];
			for( unsigned int d=1 ; d<Dim ; d++ ) os << " , " << depthAndOffset.offset[d];
			return os << " ) @ " << depthAndOffset.depth;
		}
	};
private:
	DepthAndOffsetType _depth , _offset[Dim];
	template< typename Initializer >
	bool _initChildren  ( Allocator< RegularTreeNode >* nodeAllocator , Initializer &initializer );
	template< typename Initializer >
	bool _initChildren_s( Allocator< RegularTreeNode >* nodeAllocator , Initializer &initializer );
public:

	RegularTreeNode* parent;
	RegularTreeNode* children;
	NodeData nodeData;

	RegularTreeNode( void );
	static RegularTreeNode* NewBrood( Allocator< RegularTreeNode >* nodeAllocator )
	{
		auto initializer = []( RegularTreeNode & ){};
		return NewBrood( nodeAllocator , initializer );
	}

	template< typename Initializer >
	RegularTreeNode( Initializer &initializer );
	template< typename Initializer >
	static RegularTreeNode* NewBrood( Allocator< RegularTreeNode >* nodeAllocator , Initializer &initializer );
	template< bool ThreadSafe >
	bool initChildren( Allocator< RegularTreeNode >* nodeAllocator )
	{
		auto initializer = []( RegularTreeNode & ){};
		return initChildren( nodeAllocator , initializer );
	}
	template< bool ThreadSafe , typename Initializer >
	bool initChildren( Allocator< RegularTreeNode >* nodeAllocator , Initializer &initializer )
	{
		return ThreadSafe ? _initChildren_s( nodeAllocator , initializer ) : _initChildren( nodeAllocator , initializer );
	}
	void cleanChildren( bool deleteChildren );
	static void ResetDepthAndOffset( RegularTreeNode* root , int d , int off[Dim] );
	~RegularTreeNode( void );

	// KeepNodeFunctor looks like std::function< bool ( const RegularTreeNode * ) >
	template< typename KeepNodeFunctor >
	void copySubTree( RegularTreeNode &subTree , const KeepNodeFunctor &keepNodeFunctor , Allocator< RegularTreeNode > *nodeAllocator=NULL ) const;

	template< typename KeepNodeFunctor >
	Pointer( RegularTreeNode ) serializeSubTree( const KeepNodeFunctor &keepNodeFunctor , size_t &nodeCount ) const;

	// The merge functor takes two objects of type NodeData and returns an object of type NodeData
	// [NOTE] We are assuming that the merge functor is symmetric, f(a,b) = f(b,a), and implicity satisfies f(a) = a
	template< class MergeFunctor >
	void merge( RegularTreeNode* node , MergeFunctor& f );

	void depthAndOffset( int& depth , int offset[Dim] ) const; 
	DepthAndOffset depthAndOffset( void ) const;
	void centerIndex( int index[Dim] ) const;
	int depth( void ) const;
	template< class Real > void centerAndWidth( Point< Real , Dim >& center , Real& width ) const;
	template< class Real > void startAndWidth( Point< Real , Dim >& start , Real& width ) const;
	template< class Real > bool isInside( Point< Real , Dim > p ) const;

	size_t leaves( void ) const;
	size_t maxDepthLeaves( int maxDepth ) const;
	size_t nodes( void ) const;
	int maxDepth( void ) const;

	const RegularTreeNode* root( void ) const;

	/* These functions apply the functor to the node and all descendents, terminating either at a leaf or when the functor returns false. */
	template< typename NodeFunctor /* = std::function< bool/void ( RegularTreeNode * ) > */ >
	void processNodes( NodeFunctor nodeFunctor );
	template< typename NodeFunctor /* = std::function< bool/void ( const RegularTreeNode * ) > */ >
	void processNodes( NodeFunctor nodeFunctor ) const;
	template< typename NodeFunctor /* = std::function< void ( RegularTreeNode * ) > */ >
	void processLeaves( NodeFunctor nodeFunctor );
	template< typename NodeFunctor /* = std::function< void ( const RegularTreeNode * ) > */ >
	void processLeaves( NodeFunctor nodeFunctor ) const;
protected:
	template< typename NodeFunctor /* = std::function< bool/void ( RegularTreeNode * ) > */ >
	void _processChildNodes( NodeFunctor &nodeFunctor );
	template< typename NodeFunctor /* = std::function< bool/void ( const RegularTreeNode * ) > */ >
	void _processChildNodes( NodeFunctor &nodeFunctor ) const;
	template< typename NodeFunctor /* = std::function< bool/void ( RegularTreeNode * ) > */ >
	void _processChildLeaves( NodeFunctor &nodeFunctor );
	template< typename NodeFunctor /* = std::function< bool/void ( const RegularTreeNode * ) > */ >
	void _processChildLeaves( NodeFunctor &nodeFunctor ) const;
public:

	void setFullDepth( int maxDepth , Allocator< RegularTreeNode >* nodeAllocator )
	{
		auto initializer = []( RegularTreeNode & ){};
		return setFulDepth( nodeAllocator , initializer );
	}
	template< typename Initializer >
	void setFullDepth( int maxDepth , Allocator< RegularTreeNode >* nodeAllocator , Initializer &initializer );

	template< typename PruneChildrenFunctor >
	void pruneChildren( const PruneChildrenFunctor pruneFunctor , bool deleteChildren );

	void printLeaves( void ) const;
	void printRange( void ) const;

	template< class Real > static int ChildIndex( const Point< Real , Dim >& center , const Point< Real , Dim > &p );

	// WriteNodeFunctor looks like std::function< bool ( const RegularTreeNode * ) >
	template< typename WriteNodeFunctor >
	bool write( BinaryStream &stream , bool serialize , const WriteNodeFunctor &writeNodeFunctor ) const;
	bool write( BinaryStream &stream , bool serialize ) const { return write( stream , serialize , []( const RegularTreeNode * ){ return true; } ); }

	template< typename Initializer >
	bool read( BinaryStream &stream , Allocator< RegularTreeNode >* nodeAllocator , Initializer &initializer );
	bool read( BinaryStream &stream , Allocator< RegularTreeNode >* nodeAllocator )
	{
		auto initializer = []( RegularTreeNode & ){};
		return read( stream , nodeAllocator , initializer );
	}

	template< typename Initializer >
	bool read( const char* fileName , Allocator< RegularTreeNode >* nodeAllocator , Initializer &initializer );
	template< typename Initializer >
	bool read( FILE* fp , Allocator< RegularTreeNode >* nodeAllocator , Initializer &initializer );

	template< typename Pack > struct Neighbors{};
	template< unsigned int ... Widths >
	struct Neighbors< UIntPack< Widths ... > >
	{
		typedef StaticWindow< RegularTreeNode* , UIntPack< Widths ... > > Window;
		Window neighbors;
		Neighbors( void );
		void clear( void );
	};
	template< typename Pack > struct ConstNeighbors{};
	template< unsigned int ... Widths >
	struct ConstNeighbors< UIntPack< Widths ... > >
	{
		typedef StaticWindow< const RegularTreeNode* , UIntPack< Widths ... > > Window;
		Window neighbors;
		ConstNeighbors( void );
		void clear( void );
	};

	template< typename LeftPack , typename RightPack > struct NeighborKey{};
	template< unsigned int ... LeftRadii , unsigned int ... RightRadii >
	struct NeighborKey< UIntPack< LeftRadii ... > , UIntPack< RightRadii ... > >
	{
	protected:
		static_assert( sizeof...(LeftRadii)==sizeof...(RightRadii) , "[ERROR] Left and right radii dimensions don't match" );
		static const unsigned int CenterIndex = WindowIndex< UIntPack< ( LeftRadii + RightRadii + 1 ) ... > , UIntPack< LeftRadii ... > >::Index;
		int _depth;

		template< bool CreateNodes , bool ThreadSafe , typename NodeInitializer , unsigned int ... _PLeftRadii , unsigned int ... _PRightRadii , unsigned int ... _CLeftRadii , unsigned int ... _CRightRadii >
		static unsigned int _NeighborsLoop( UIntPack< _PLeftRadii ... > , UIntPack< _PRightRadii ... > , UIntPack< _CLeftRadii ... > , UIntPack< _CRightRadii ... > , ConstWindowSlice< RegularTreeNode* , UIntPack< ( _PLeftRadii+_PRightRadii+1 ) ... > > pNeighbors , WindowSlice< RegularTreeNode* , UIntPack< ( _CLeftRadii+_CRightRadii+1 ) ... > > cNeighbors , int cIdx , Allocator< RegularTreeNode >* nodeAllocator , NodeInitializer &initializer );

		template< bool CreateNodes , bool ThreadSafe , typename NodeInitializer , unsigned int ... _PLeftRadii , unsigned int ... _PRightRadii , unsigned int ... _CLeftRadii , unsigned int ... _CRightRadii >
		static unsigned int _NeighborsLoop( UIntPack< _PLeftRadii ... > , UIntPack< _PRightRadii ... > , UIntPack< _CLeftRadii ... > , UIntPack< _CRightRadii ... > ,      WindowSlice< RegularTreeNode* , UIntPack< ( _PLeftRadii+_PRightRadii+1 ) ... > > pNeighbors , WindowSlice< RegularTreeNode* , UIntPack< ( _CLeftRadii+_CRightRadii+1 ) ... > > cNeighbors , int cIdx , Allocator< RegularTreeNode >* nodeAllocator , NodeInitializer &initializer );

		template< bool CreateNodes , bool ThreadSafe , typename NodeInitializer , typename PLeft , typename PRight , typename CLeft , typename CRight > struct _Run{};

		template< bool CreateNodes , bool ThreadSafe , typename NodeInitializer , unsigned int _PLeftRadius , unsigned int ... _PLeftRadii , unsigned int _PRightRadius , unsigned int ... _PRightRadii , unsigned int _CLeftRadius , unsigned int ... _CLeftRadii , unsigned int _CRightRadius , unsigned int ... _CRightRadii >
		struct _Run< CreateNodes , ThreadSafe , NodeInitializer , UIntPack< _PLeftRadius , _PLeftRadii ... > , UIntPack< _PRightRadius , _PRightRadii ... > , UIntPack< _CLeftRadius , _CLeftRadii ... > , UIntPack< _CRightRadius , _CRightRadii ... > >
		{
			static unsigned int Run( ConstWindowSlice< RegularTreeNode* , UIntPack< _PLeftRadius+_PRightRadius+1 , ( _PLeftRadii+_PRightRadii+1 ) ... > > pNeighbors , WindowSlice< RegularTreeNode* , UIntPack< _CLeftRadius+_CRightRadius+1 , ( _CLeftRadii+_CRightRadii+1 ) ... > > cNeighbors , int* c , int cornerIndex , Allocator< RegularTreeNode >* nodeAllocator , NodeInitializer &initializer );
		};
		template< bool CreateNodes , bool ThreadSafe , typename NodeInitializer , unsigned int _PLeftRadius , unsigned int _PRightRadius , unsigned int _CLeftRadius , unsigned int _CRightRadius >
		struct _Run< CreateNodes , ThreadSafe , NodeInitializer , UIntPack< _PLeftRadius > , UIntPack< _PRightRadius > , UIntPack< _CLeftRadius > , UIntPack< _CRightRadius > >
		{
			static unsigned int Run( ConstWindowSlice< RegularTreeNode* , UIntPack< _PLeftRadius+_PRightRadius+1 > > pNeighbors , WindowSlice< RegularTreeNode* , UIntPack< _CLeftRadius+_CRightRadius+1 > > cNeighbors , int* c , int cornerIndex , Allocator< RegularTreeNode >* nodeAllocator , NodeInitializer &initializer );
		};
	public:
		typedef Neighbors< UIntPack< ( LeftRadii + RightRadii + 1 ) ... > > NeighborType;
		NeighborType* neighbors;


		NeighborKey( void );
		NeighborKey( const NeighborKey& key );
		~NeighborKey( void );
		int depth( void ) const { return _depth; }

		void set( int depth );

		template< bool CreateNodes , bool ThreadSafe >
		typename RegularTreeNode< Dim , NodeData , DepthAndOffsetType >::template Neighbors< UIntPack< ( LeftRadii + RightRadii + 1 ) ... > >& getNeighbors( RegularTreeNode* node , Allocator< RegularTreeNode >* nodeAllocator )
		{
			auto initializer = []( RegularTreeNode & ){};
			return getNeighbors( node , nodeAllocator , initializer );
		}

		template< bool CreateNodes , bool ThreadSafe , typename NodeInitializer >
		typename RegularTreeNode< Dim , NodeData , DepthAndOffsetType >::template Neighbors< UIntPack< ( LeftRadii + RightRadii + 1 ) ... > >& getNeighbors( RegularTreeNode* node , Allocator< RegularTreeNode >* nodeAllocator , NodeInitializer &nodeInitializer );

		NeighborType& getNeighbors( const RegularTreeNode* node )
		{
			auto initializer = []( RegularTreeNode & ){};
			return getNeighbors< false , false >( (RegularTreeNode*)node , NULL , initializer );
		}

		template< bool CreateNodes , bool ThreadSafe , unsigned int ... _LeftRadii , unsigned int ... _RightRadii >
		void getNeighbors( UIntPack< _LeftRadii ... > , UIntPack< _RightRadii ... > ,       RegularTreeNode* node , Neighbors< UIntPack< ( _LeftRadii + _RightRadii + 1 ) ... > >& neighbors , Allocator< RegularTreeNode >* nodeAllocator )
		{
			auto initializer = []( RegularTreeNode & ){};
			return getNeighbors( UIntPack< _LeftRadii ... >() , UIntPack< _RightRadii ... >() , node , neighbors , nodeAllocator , initializer );
		}

		template< bool CreateNodes , bool ThreadSafe , typename NodeInitializer , unsigned int ... _LeftRadii , unsigned int ... _RightRadii >
		void getNeighbors( UIntPack< _LeftRadii ... > , UIntPack< _RightRadii ... > ,       RegularTreeNode* node , Neighbors< UIntPack< ( _LeftRadii + _RightRadii + 1 ) ... > >& neighbors , Allocator< RegularTreeNode >* nodeAllocator , NodeInitializer &initializer );

		template< unsigned int ... _LeftRadii , unsigned int ... _RightRadii >
		void getNeighbors( UIntPack< _LeftRadii ... > , UIntPack< _RightRadii ... > , const RegularTreeNode* node , Neighbors< UIntPack< ( _LeftRadii + _RightRadii + 1 ) ... > >& neighbors )
		{
			auto initializer = []( RegularTreeNode & ){};
			return getNeighbors< false , false >( UIntPack< _LeftRadii ... >() , UIntPack< _RightRadii ... >() , (RegularTreeNode*)node , NULL , initializer );
		}

		template< bool CreateNodes , bool ThreadSafe , unsigned int ... _LeftRadii , unsigned int ... _RightRadii >
		void getNeighbors( UIntPack< _LeftRadii ... > , UIntPack< _RightRadii ... > ,       RegularTreeNode* node , Neighbors< UIntPack< ( _LeftRadii + _RightRadii + 1 ) ... > >& pNeighbors , Neighbors< UIntPack< ( _LeftRadii + _RightRadii + 1 ) ... > >& neighbors , Allocator< RegularTreeNode >* nodeAllocator )
		{
			auto initializer = []( RegularTreeNode & ){};
			return getNeighbors( UIntPack< _LeftRadii ... >() , UIntPack< _RightRadii ... >() , node , pNeighbors , neighbors , nodeAllocator , initializer );
		}

		template< bool CreateNodes , bool ThreadSafe , typename NodeInitializer , unsigned int ... _LeftRadii , unsigned int ... _RightRadii >
		void getNeighbors( UIntPack< _LeftRadii ... > , UIntPack< _RightRadii ... > ,       RegularTreeNode* node , Neighbors< UIntPack< ( _LeftRadii + _RightRadii + 1 ) ... > >& pNeighbors , Neighbors< UIntPack< ( _LeftRadii + _RightRadii + 1 ) ... > >& neighbors , Allocator< RegularTreeNode >* nodeAllocator , NodeInitializer &initializer );

		template< unsigned int ... _LeftRadii , unsigned int ... _RightRadii >
		void getNeighbors( UIntPack< _LeftRadii ... > , UIntPack< _RightRadii ... > , const RegularTreeNode* node , Neighbors< UIntPack< ( _LeftRadii + _RightRadii + 1 ) ... > >& pNeighbors , Neighbors< UIntPack< ( _LeftRadii + _RightRadii + 1 ) ... > >& neighbors )
		{
			auto initializer = []( RegularTreeNode & ){};
			return getNeighbors< false , false >( UIntPack< _LeftRadii ... >() , UIntPack< _RightRadii ... >() , (RegularTreeNode*)node , NULL , initializer );
		}

		template< bool CreateNodes , bool ThreadSafe >
		unsigned int getChildNeighbors( int cIdx , int d , NeighborType& childNeighbors , Allocator< RegularTreeNode >* nodeAllocator ) const
		{
			auto initializer = []( RegularTreeNode & ){};
			return getChildNeighbors( cIdx , d , childNeighbors , nodeAllocator , initializer );
		}

		template< bool CreateNodes , bool ThreadSafe , typename NodeInitializer >
		unsigned int getChildNeighbors( int cIdx , int d , NeighborType& childNeighbors , Allocator< RegularTreeNode >* nodeAllocator , NodeInitializer &initializer ) const;

		unsigned int getChildNeighbors( int cIdx , int d , NeighborType& childNeighbors ) const
		{
			auto initializer = []( RegularTreeNode & ){};
			return getChildNeighbors< false , false >( cIdx , d , childNeighbors , NULL , initializer );
		}

		template< bool CreateNodes , bool ThreadSafe , class Real >
		unsigned int getChildNeighbors( Point< Real , Dim > p , int d , NeighborType& childNeighbors , Allocator< RegularTreeNode >* nodeAllocator ) const
		{
			auto initializer = []( RegularTreeNode & ){};
			return getChildNeighbors( p , d , childNeighbors , nodeAllocator , initializer );
		}

		template< bool CreateNodes , bool ThreadSafe , typename NodeInitializer , class Real >
		unsigned int getChildNeighbors( Point< Real , Dim > p , int d , NeighborType& childNeighbors , Allocator< RegularTreeNode >* nodeAllocator , NodeInitializer &initializer ) const;

		template< class Real >
		unsigned int getChildNeighbors( Point< Real , Dim > p , int d , NeighborType& childNeighbors ) const
		{
			auto initializer = []( RegularTreeNode & ){};
			return getChildNeighbors< false , false , Real >( p , d , childNeighbors , NULL , initializer );
		}

		void setLeafNeighbors( RegularTreeNode *node , StaticWindow< RegularTreeNode * , UIntPack< ( LeftRadii + RightRadii + 1 ) ... > > &leaves );
	};

	template< typename LeftPack , typename RightPack > struct ConstNeighborKey{};

	template< unsigned int ... LeftRadii , unsigned int ... RightRadii >
	struct ConstNeighborKey< UIntPack< LeftRadii ... > , UIntPack< RightRadii ... > >
	{
	protected:
		static_assert( sizeof...(LeftRadii)==sizeof...(RightRadii) , "[ERROR] Left and right radii dimensions don't match" );
		static const unsigned int CenterIndex = WindowIndex< UIntPack< ( LeftRadii + RightRadii + 1 ) ... > , UIntPack< LeftRadii ... > >::Index;
		int _depth;

		template< unsigned int ... _PLeftRadii , unsigned int ... _PRightRadii , unsigned int ... _CLeftRadii , unsigned int ... _CRightRadii >
		static unsigned int _NeighborsLoop( UIntPack< _PLeftRadii ... > , UIntPack< _PRightRadii ... > , UIntPack< _CLeftRadii ... > , UIntPack< _CRightRadii ... > , ConstWindowSlice< const RegularTreeNode* , UIntPack< ( _PLeftRadii+_PRightRadii+1 ) ... > > pNeighbors , WindowSlice< const RegularTreeNode* , UIntPack< ( _CLeftRadii+_CRightRadii+1 ) ... > > cNeighbors , int cIdx );
		template< unsigned int ... _PLeftRadii , unsigned int ... _PRightRadii , unsigned int ... _CLeftRadii , unsigned int ... _CRightRadii >
		static unsigned int _NeighborsLoop( UIntPack< _PLeftRadii ... > , UIntPack< _PRightRadii ... > , UIntPack< _CLeftRadii ... > , UIntPack< _CRightRadii ... > , WindowSlice< const RegularTreeNode* , UIntPack< ( _PLeftRadii+_PRightRadii+1 ) ... > > pNeighbors , WindowSlice< const RegularTreeNode* , UIntPack< ( _CLeftRadii+_CRightRadii+1 ) ... > > cNeighbors , int cIdx );

		template< typename PLeft , typename PRight , typename CLeft , typename CRight > struct _Run{};

		template< unsigned int _PLeftRadius , unsigned int ... _PLeftRadii , unsigned int _PRightRadius , unsigned int ... _PRightRadii , unsigned int _CLeftRadius , unsigned int ... _CLeftRadii , unsigned int _CRightRadius , unsigned int ... _CRightRadii >
		struct _Run< UIntPack< _PLeftRadius , _PLeftRadii ... > , UIntPack< _PRightRadius , _PRightRadii ... > , UIntPack< _CLeftRadius , _CLeftRadii ... > , UIntPack< _CRightRadius , _CRightRadii ... > >
		{
			static unsigned int Run( ConstWindowSlice< const RegularTreeNode* , UIntPack< _PLeftRadius + _PRightRadius + 1 , ( _PLeftRadii+_PRightRadii+1 ) ... > > pNeighbors , WindowSlice< const RegularTreeNode* , UIntPack< _CLeftRadius + _CRightRadius + 1 , ( _CLeftRadii+_CRightRadii+1 ) ... > > cNeighbors , int* c , int cornerIndex );
		};
		template< unsigned int _PLeftRadius , unsigned int _PRightRadius , unsigned int _CLeftRadius , unsigned int _CRightRadius >
		struct _Run< UIntPack< _PLeftRadius > , UIntPack< _PRightRadius > , UIntPack< _CLeftRadius > , UIntPack< _CRightRadius > >
		{
			static unsigned int Run( ConstWindowSlice< const RegularTreeNode* , UIntPack< _PLeftRadius+_PRightRadius+1 > > pNeighbors , WindowSlice< const RegularTreeNode* , UIntPack< _CLeftRadius+_CRightRadius+1 > > cNeighbors , int* c , int cornerIndex );
		};

	public:

		typedef ConstNeighbors< UIntPack< ( LeftRadii + RightRadii + 1 ) ... > > NeighborType;
		NeighborType* neighbors;

		ConstNeighborKey( void );
		ConstNeighborKey( const ConstNeighborKey& key );
		~ConstNeighborKey( void );
		ConstNeighborKey& operator = ( const ConstNeighborKey& key );

		int depth( void ) const { return _depth; }
		void set( int depth );

		typename RegularTreeNode< Dim , NodeData , DepthAndOffsetType >::template ConstNeighbors< UIntPack< ( LeftRadii + RightRadii + 1 ) ... > >& getNeighbors( const RegularTreeNode* node );
		template< unsigned int ... _LeftRadii , unsigned int ... _RightRadii >
		void getNeighbors( UIntPack< _LeftRadii ... > , UIntPack< _RightRadii ... > , const RegularTreeNode* node , ConstNeighbors< UIntPack< ( _LeftRadii + _RightRadii + 1 ) ... > >& neighbors );
		template< unsigned int ... _LeftRadii , unsigned int ... _RightRadii >
		void getNeighbors( UIntPack< _LeftRadii ... > , UIntPack< _RightRadii ... > , const RegularTreeNode* node , ConstNeighbors< UIntPack< ( _LeftRadii + _RightRadii + 1 ) ... > >& pNeighbors , ConstNeighbors< UIntPack< ( _LeftRadii + _RightRadii + 1 ) ... > >& neighbors );
		unsigned int getChildNeighbors( int cIdx , int d , NeighborType& childNeighbors ) const;
		template< class Real >
		unsigned int getChildNeighbors( Point< Real , Dim > p , int d , ConstNeighbors< UIntPack< ( LeftRadii + RightRadii + 1 ) ... > >& childNeighbors ) const;

		void setLeafNeighbors( const RegularTreeNode *node , StaticWindow< RegularTreeNode * , UIntPack< ( LeftRadii + RightRadii + 1 ) ... > > &leaves );
	};

	int width( int maxDepth ) const;
};

#include "RegularTree.inl"

#endif // REGULAR_TREE_NODE_INCLUDED
