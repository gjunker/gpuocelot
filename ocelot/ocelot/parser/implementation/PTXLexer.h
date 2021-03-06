/*!
	\file PTXLexer.h
	\date Monday January 19, 2009
	\author Gregory Diamos <gregory.diamos@gatech.edu>
	\brief The header file for the PTXLexer class.
*/

#ifndef PTX_LEXER_H_INCLUDED
#define PTX_LEXER_H_INCLUDED

union YYSTYPE;

//#if defined(ptxFlexLexer)
//#undef yyFlexLexer
//#define yyFlexLexer ptxFlexLexer
//#endif
#include <FlexLexer.h>

namespace parser
{
	/*!	\brief A wrapper around yyFlexLexer to allow for a local variable */
    class PTXLexer : public yyFlexLexer
	{
		public:
			YYSTYPE*     yylval;
			int          column;
			int          nextColumn;

		public:
			PTXLexer( std::istream* arg_yyin = 0, 
				std::ostream* arg_yyout = 0 );
	
			int yylex();
			int yylexPosition();
			
		public:
			static std::string toString( int token );
	
	};

}

#endif

