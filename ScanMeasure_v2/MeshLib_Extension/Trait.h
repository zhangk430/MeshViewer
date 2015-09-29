#ifndef _XMESHLIB_CORE_TRAIT_H_
#define _XMESHLIB_CORE_TRAIT_H_

#pragma warning (disable : 4786)
#include <list>
#include <algorithm>



namespace XMeshLib
{
	class TraitNode
	{
	public:
		TraitNode(){tId = -1;}
		virtual ~TraitNode(){tId = -1;}
		bool operator== (const TraitNode & t) const
		{
			int tid1 = t.tId;			
			if (tId == tid1)
				return true;
			else
				return false;			
		}
		int tId;
	};	

	class TraitList
	{
	public:
		TraitList(){;}
		~TraitList(){clear();}
		void clear()
		{
			for (std::list<TraitNode *>::iterator titer=tlist.begin(); titer!=tlist.end(); ++titer)
			{				
				TraitNode * tempTrait = * titer;				
				delete tempTrait;
			}
			tlist.clear();
		}
		void addTrait(TraitNode * t)
		{
			tlist.push_back(t);
		}
		bool delTrait(int id)
		{
			//TraitNode dummyTrait;
			//dummyTrait.tId = id;
			//std::list<TraitNode *>::iterator titer = std::find(tlist.begin(), tlist.end(), &dummyTrait);
			//if (titer!=tlist.end())
			//{
			//	TraitNode * tempTrait = *titer;				
			//	tlist.erase(titer);
			//	delete tempTrait;
			//	return true;
			//}
			//else
			//	return false;
			std::list<TraitNode *>::iterator titer;
			for (titer=tlist.begin(); titer!=tlist.end(); ++titer)
			{				
				TraitNode * tempTrait = * titer;
				if (id == tempTrait->tId)
				{
					tlist.erase(titer);
					delete tempTrait;
					return true;
				}
			}
			return false;
		}
		TraitNode * getTrait(int id)
		{
			std::list<TraitNode *>::iterator titer;
			for (titer=tlist.begin(); titer!=tlist.end(); ++titer)
			{				
				TraitNode * tempTrait = * titer;
				if (id == tempTrait->tId)
					return tempTrait;
			}
			return NULL;
		}
		std::list<TraitNode *> tlist;		
	};
}

#endif

