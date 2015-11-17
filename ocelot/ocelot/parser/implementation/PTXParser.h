/*! \file PTXParser.h
	\date Monday January 19, 2009
	\author Gregory Diamos <gregory.diamos@gatech.edu>
	\brief The header file for the PTXParser class.
*/

#ifndef PTX_PARSER_H_INCLUDED
#define PTX_PARSER_H_INCLUDED

#include <ocelot/parser/interface/Parser.h>
#include <ocelot/ir/interface/Module.h>

#include "PTXParserState.h"
#include "PTXLexer.h"

namespace parser
{
	/*! \brief An implementation of the Parser interface for PTX */
	class PTXParser : public Parser
	{
		private:
            PTXParserState state;
		
		private:
			void checkLabels();
			void reset();
			std::string getLinesNearCurrentLocation( std::istream& input );
		
		public:
//			static ir::PTXOperand::DataType tokenToDataType( int );
//			static ir::PTXOperand::VectorIndex tokenToVectorIndex( int );
//			static ir::PTXInstruction::Vec tokenToVec( int );
//			static ir::PTXInstruction::Opcode stringToOpcode( std::string );
//			static ir::PTXOperand::SpecialRegister
//				stringToSpecial( std::string );
//			static ir::PTXInstruction::Modifier tokenToModifier( int );
//			static ir::PTXInstruction::AddressSpace tokenToAddressSpace( int );
//			static ir::PTXStatement::Directive tokenToDirective( int );
//			static ir::PTXInstruction::ReductionOperation
//				tokenToReductionOperation( int );
//			static ir::PTXInstruction::AtomicOperation
//				tokenToAtomicOperation( int );
//			static ir::PTXInstruction::CmpOp tokenToCmpOp( int );
//			static ir::PTXInstruction::BarrierOperation tokenToBarrierOp(int);
//			static ir::PTXInstruction::CacheOperation tokenToCacheOperation(int);
//			static ir::PTXInstruction::CacheLevel tokenToCacheLevel(int);
//			static ir::PTXInstruction::ClampOperation tokenToClampOperation(int);
//			static ir::PTXInstruction::FormatMode tokenToFormatMode(int);
//			static ir::PTXInstruction::SurfaceQuery tokenToSurfaceQuery(int);
//			static ir::PTXInstruction::ColorComponent
//				tokenToColorComponent(int);
//			static ir::PTXInstruction::BoolOp tokenToBoolOp( int );
//			static ir::PTXInstruction::Geometry tokenToGeometry( int );
//			static ir::PTXInstruction::VoteMode tokenToVoteMode( int );
//			static ir::PTXInstruction::ShuffleMode tokenToShuffleMode( int );
//			static ir::PTXInstruction::Level tokenToLevel( int );
//			static ir::PTXInstruction::PermuteMode tokenToPermuteMode( int );
//			static ir::PTXInstruction::FloatingPointMode
//				tokenToFloatingPointMode( int);
//			static ir::PTXStatement::TextureSpace tokenToTextureSpace( int );
//			static ir::PTXOperand::DataType smallestType( long long int );
//			static ir::PTXOperand::DataType
//				smallestType( long long unsigned int );
			
		public:
			PTXParser();
			void parse( std::istream& input, 
				ir::Instruction::Architecture language = ir::Instruction::PTX );
			ir::Module::StatementVector&& statements();	
	};

}

#endif

