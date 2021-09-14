package kueblerWall;
// written by Eric Kuebler Jan 17 2021
import battlecode.common.*;
import java.lang.Math;
import java.util.*;
class Path {
  /*
    // Class Author: Eric Kuebler
    * This class provides full pathfinding
    capabilities in simple and complex
    situations
  */
  static MapLocation target;
  static int targetDistance;
  static Queue<Integer> path;
  static int pathSize;
  static boolean done;
  static boolean edge;
  static boolean stuck;
  static int stuckCount;
  static MapLocation dataIDStandard;
  static void initialize() {
    target = new MapLocation(0, 0);
    targetDistance = 0;
    path = new LinkedList<Integer>();
    pathSize = 0;
    done = true;
    edge = false;
    stuck = false;
    stuckCount = 0;
    dataIDStandard = new MapLocation(RobotPlayer.posX - 64, RobotPlayer.posY - 64);
  }
  static void setNewTarget(int x, int y) {
    if(done) {
      done = false;
    }
    if(edge) {
      edge = false;
    }
    if(stuck) {
      stuck = false;
    }
    clearPath();
    target = new MapLocation(x, y);
  }
  static boolean atTarget(int x, int y) {
    return x == target.x && y == target.y;
  }
  static void addToPath(int dir) {
    path.add(dir);
    ++pathSize;
  }
  static void clearPath() {
    while(pathSize-- > 0) {
      path.remove();
    }
    pathSize = 0;
  }
  static void handleMovement() {
    //System.out.println("target: " + target + "\npathSize: " + pathSize + "\ndone: " + done + "\nedge: " + edge + "\nstuck: " + stuck);
    if(done || stuck) {
      return;
    }
    if(atTarget(RobotPlayer.posX, RobotPlayer.posY)) {
      // at location
      done = true;
      targetDistance = 0;
      return;
    }
    else if(pathSize == 0) { // needs a new path
      buildFastPath();
    }
    try {
      if(RobotPlayer.rc.isReady()) {
        if(pathSize != 0 && RobotPlayer.tryMove(RobotPlayer.directions[(int)path.peek()])) { // successful movement
          path.remove(); // remove last item in path
          --pathSize;
          targetDistance = Path.distance(RobotPlayer.posX - target.x, RobotPlayer.posY - target.y);
          if(stuckCount > 0) {
            stuckCount = 0;
          }
          if(atTarget(RobotPlayer.posX, RobotPlayer.posY)) {
            done = true;
            targetDistance = 0;
            clearPath();
          }
          else if(pathSize == 0) {
            buildFastPath();
          }
        }
        else { // attempt building a better path
          buildObstructionPath();
          if(pathSize == 0) {
            stuck = true;
            if(++stuckCount > 30) { // send signal to spread local MBs
              final int threatCount = RobotPlayer.rc.senseNearbyRobots(8, RobotPlayer.team == Team.A ? Team.B : Team.A).length;
              if(threatCount == 0) {
                RobotPlayer.setFlag(10, new Flag(10, 32, -1, -1, -1)); // spread protocol
              }
            }
          }
        }
      }
    }
    catch(Exception e) {
      System.out.println("Path.handleMovement Exception");
      e.printStackTrace();
    }
  }
  static void buildFastPath() {
    /* 
      * this function will build a path from
      the current location to the farthest 
      sensor distance location using the 
      lowest amount of computation while 
      still checking for best passability
      
      * does NOT check for obstructions
    */
    try {
      int curX = RobotPlayer.posX;
      int curY = RobotPlayer.posY;
      clearPath();
      int dx;
      int dy;
      while(!edge && pathSize < 4 && (curX != target.x || curY != target.y)) {
        dx = target.x - curX;
        dy = target.y - curY;
        if(distance(dx, dy) <= 2) {
          addToPath(dirInt(dx, dy));
          return;
        }
        int best = -2;
        double bestCost = 0;
        if(Math.abs(dx) > Math.abs(dy)) { // horizontal
          dx = (int)Math.signum(dx); 
          for(dy = -1; dy < 2; ++dy) {
            final double pass = safePassability(curX + dx, curY + dy);
            if(pass == -1) {
              continue;
            }
            final double cost = distance(target.x - curX - dx, target.y - curY - dy) - 6 * pass;
            if(best == -2 || cost < bestCost) {
              best = dy;
              bestCost = cost;
            }
          }
          if(best == -2) {
            edge = true;
          }
          else {
            addToPath(dirInt(dx, best));
            curX += dx;
            curY += best;
          }
        }
        else { // vertical
          dy = (int)Math.signum(dy);
          for(dx = -1; dx < 2; ++dx) {
            final double pass = safePassability(curX + dx, curY + dy);
            if(pass == -1) {
              continue;
            }
            final double cost = distance(target.x - curX - dx, target.y - curY - dy) - 6 * pass;
            if(best == -2 || cost < bestCost) {
              best = dx;
              bestCost = cost;
            }
          }
          if(best == -2) {
            edge = true;
          }
          else {
            addToPath(dirInt(best, dy));
            curX += best;
            curY += dy;
          }
        }
      }
    }
    catch(Exception e) {
      System.out.println("Path.buildFastPath failed");
    }
  }
  static void buildObstructionPath() {
    /*
      This function is used to find a path
      when the FastPath is obstructed

      * WARNING: EXPENSIVE

      * This function ignores passability entirely

    */
    try {
      Hashtable<Integer, Path> data = new Hashtable<Integer, Path>();
      int curX = RobotPlayer.posX;
      int curY = RobotPlayer.posY;
      final int first = locationToDataID(curX, curY);
      int parent = first;
      int closest = first;
      data.put(parent, new Path(-1, curX, curY, 0, distance(curX - target.x, curY - target.y), 0));
      boolean found = false;
      boolean failure = false;
      while(!found && !failure) {
        curX = data.get(parent).x;
        curY = data.get(parent).y;
        data.get(parent).checked = true;
        final int pathDist = data.get(parent).pathDist + 1;
        if(pathDist == 4) {
          break;
        }
        for(int dy = -1; !found && dy < 2; ++dy) {
          for(int dx = -1; !found && dx < 2; ++dx) {
            if(dx == 0 && dy == 0) {
              continue;
            }
            /*
              the following code is structured
              in a strange way to reduce
              the amount of times safeObstructed
              is called because it is expensive
            */
            final int dataID = locationToDataID(curX + dx, curY + dy);
            if(data.get(dataID) != null) { // location already present in hashtable
              if(pathDist < data.get(dataID).pathDist) {
                data.get(dataID).parent = parent;
                data.get(dataID).pathDist = pathDist;
                data.get(dataID).checked = false;
              }
            }
            else {
              final int occupied = safeOccupied(curX + dx, curY + dy);
              if(occupied == 1) { // location can be moved to
                final int dist = distance(curX + dx - target.x, curY + dy - target.y);
                data.put(dataID, new Path(parent, curX + dx, curY + dy, pathDist, dist, dirInt(dx, dy)));
                if(dist < data.get(closest).tarDist) {
                  closest = dataID;
                }
              }
              else if(occupied == 0) {
                data.get(parent).nearEdge = true;
              }
              if(atTarget(curX + dx, curY + dy)) {
                found = true;
              }
            }
          }
        }
        if(!found) {
          int nextDataID = -1;
          Set<Integer> keys = data.keySet();
          for(int key : keys) {
            if(!data.get(key).checked && (nextDataID == -1 || data.get(key).tarDist < data.get(nextDataID).tarDist)) {
              nextDataID = key;
            }
          }
          if(nextDataID == -1) {
            failure = true;
          }
          else {
            parent = nextDataID;
          }
        }
      }
      clearPath();
      ArrayList<Integer> reversePath = new ArrayList<Integer>();
      while(closest != first) {
        reversePath.add(data.get(closest).dir);
        if(data.get(closest).nearEdge) {
          edge = true;
        }
        closest = data.get(closest).parent;
      }
      for(int i = reversePath.size() - 1; i > -1; --i) {
        addToPath(reversePath.get(i));
      }
    }
    catch(Exception e) {
      System.out.println("Path.buildObstructedPath failed");
    }
  }
  static double safePassability(int x, int y) {
    try {
      final MapLocation location = new MapLocation(x, y);
      if(RobotPlayer.rc.canSenseLocation(location)) {
        return RobotPlayer.rc.sensePassability(location);
      }
    }
    catch(Exception e) {
      System.out.println("safePassability failed");
    }
    return -1;
  }
  static int safeOccupied(int x, int y) {
    // returns 1 if location is NOT obstructed
    // returns 0 if not in map
    try {
      final MapLocation location = new MapLocation(x, y);
      if(RobotPlayer.rc.canSenseLocation(location)) { // cost 5
        return RobotPlayer.rc.isLocationOccupied(location) ? -1 : 1; // cost 20
      }
      else if(!RobotPlayer.rc.onTheMap(location)) { // cost 5
        return 0;
      }
    }
    catch(Exception e) {
      System.out.println("safeObstructed failed");
    }
    return -1;
  }
  static int dirInt(int dx, int dy) {
    /* 
      this function returns an integer correlating
      to a direction in the RobotPlayer.directions array
    */
    int direction = 0;
    switch(dy) {
      case -1:
        switch(dx) {
          case -1: direction = 5; break;
          case 0:  direction = 4; break;
          case 1:  direction = 3; break;
        }
      break;
      case 0:
        switch(dx) {
          case -1: direction = 6; break;
          case 1:  direction = 2; break;
        }
      break;
      case 1:
        switch(dx) {
          case -1: direction = 7; break;
          case 1:  direction = 1; break;
        }
      break;
    }
    return direction;
  }
  static int distance(int dx, int dy) {
    return dx * dx + dy * dy;
  }
  static int locationToDataID(int x, int y) {
    return 128 * (y - dataIDStandard.y) + (x - dataIDStandard.x);
  }
  static MapLocation dataIDToLocation(int dataID) {
    return new MapLocation((int)Math.floor(dataID / 128) + dataIDStandard.x, (dataID % 128) + dataIDStandard.y);
  }
  int parent;
  int x;
  int y;
  int pathDist;
  int tarDist;
  int dir;
  boolean checked = false;
  boolean nearEdge = false;
  Path(int in_parent, int in_x, int in_y, int in_pathDist, int in_tarDist, int in_dir) {
    parent = in_parent;
    x = in_x;
    y = in_y;
    pathDist = in_pathDist;
    tarDist = in_tarDist;
    dir = in_dir;
  }
}
class Flag {
  /*
    // Class Author: Eric Kuebler
    * this class provides useful functionality
      for encoding and decoding information into
      the 24 bit flag values

    * frequency determines the bit length of the message

    * frequencies 21 through 24 are reserved for
      locations with conviction values
  
    * frequency 20 references a location offset
      from a fixed point position, allowing for
      a coordinate to be interpreted by
      every robot without knowing a prior location

    * frequency 19 encodes a large integer [10000, 42767]
      typically used for sending IDs

    * frequency 17 is used for chaining threat data
      relative to a robot's location
  */
  static final Flag failed = new Flag(-1, -1, -1, -1, -1);
  static Flag decode(int input) {
    if(input > 16777215) {
      return failed;
    }
    final int frequency = log2(input);
    Flag decodedFlag = failed;
    switch(frequency) {
      case 24:
      case 23:
      case 22:
      case 21:
        final int freqOffset = 24 - frequency;
        decodedFlag = new Flag(
          frequency,
          // signal
          decodeSubstr(frequency, input, 1, 2),
          // conviction
          decodeSubstr(frequency, input, 3, 7 + freqOffset) << (4 - freqOffset),
          // x
          decodeSubstr(frequency, input, 10 + freqOffset, 7 - freqOffset) - (63 >> freqOffset),
          // y
          decodeSubstr(frequency, input, 17, 7 - freqOffset) - (63 >> freqOffset)
        ); 
      break;
      case 20:
        try {
          final MapLocation fixed = RobotPlayer.fixedLocation(
            decodeSubstr(frequency, input, 4, 8),
            decodeSubstr(frequency, input, 12, 8)
          );
          decodedFlag = new Flag(
            frequency,
            // signal
            decodeSubstr(frequency, input, 1, 3),
            fixed.x,
            fixed.y,
            -1
          );
        }
        catch(Exception e) {
          System.out.println("Flag Frequency 20 Exception");
          e.printStackTrace();
        }
      break;
      case 19:
        decodedFlag = new Flag(
          frequency,
          // signal
          decodeSubstr(frequency, input, 1, 3),
          // data
          decodeSubstr(frequency, input, 4, 15) + 10000,
          -1,
          -1
        );
      break;
      case 17:
        decodedFlag = new Flag(
          frequency,
          // signal
          decodeSubstr(frequency, input, 1, 2),
          // x
          decodeSubstr(frequency, input, 3, 7) - 63,
          // y
          decodeSubstr(frequency, input, 10, 7) - 63,
          -1
        );
      break;
      case 11:
        try {
          final MapLocation fixed = RobotPlayer.fixedLocation(
            decodeSubstr(frequency, input, 3, 8),
            decodeSubstr(frequency, input, 3, 8)
          );
          final int xory = decodeSubstr(frequency, input, 2, 1);
          decodedFlag = new Flag(
            frequency,
            // signal
            decodeSubstr(frequency, input, 1, 1),
            // x || y
            xory,
            // data
            xory == 1 ? fixed.y : fixed.x,
            -1
          );
        }
        catch(Exception e) {
          System.out.println("Flag Frequency 11 Exception");
        }
      break;
      case 10:
        decodedFlag = new Flag(
          frequency,
          // signal
          decodeSubstr(frequency, input, 1, 9),
          -1,
          -1,
          -1
        );
      break;
      case 9:
        decodedFlag = new Flag(
          frequency,
          -1,
          // data
          decodeSubstr(frequency, input, 1, 8),
          -1,
          -1
        );
      break;
    }
    return decodedFlag;
  }
  static int decodeSubstr(int frequency, int input, int start, int length) {
    return (input >> (frequency - start - length)) % (1 << length);
  }
  static int log2(int x) {
    return x == 0 ? 0 : (int)Math.ceil(Math.log(x) / Math.log(2));
  }
  int frequency;
  int signal;
  int data = -1;
  int x = -1;
  int y = -1;
  Flag(int in_frequency, int in_signal, int a, int b, int c) {
    frequency = in_frequency;
    signal = in_signal;
    switch(frequency) {
      case 24:
      case 23:
      case 22:
      case 21:
        data = a;
        x = b;
        y = c;
        // downgrade frequency if applicable
        if(frequency == 24) {
          final int constraint = 6 - log2(Math.max(Math.abs(b) + (b < 0 ? 1 : 0), Math.abs(c) + (c < 0 ? 1 : 0)));
          if(constraint > 3) {
            frequency = 21;
          }
          else {
            frequency -= constraint;
          }
        }
      break;
      case 20:
      case 17:
        x = a;
        y = b;
      break;
      case 19:
        data = a;
      break;
      case 11:
        y = a;
        data = b;
      break;
      case 10:
        // purely a signal channel
      break;
      case 9:
        data = a;
      break;
    }
    //System.out.println("(constructed Flag): freq: " + frequency + ", sig: " + signal + ", data: " + data + ", (" + x + ", " + y + ")");
  }
  int encode() {
    int base = 1 << (frequency - 1);
    switch(frequency) {
      case 24:
      case 23:
      case 22:
      case 21:
        if(data < 0) {
          data = 0;
        }
        final int freqOffset = 24 - frequency;
        // signal
        base += encodeSubstr(signal, 1, 2);
        // conviction
        base += encodeSubstr(Math.min(data >> (4 - freqOffset), (1 << (7 + freqOffset)) - 1), 3, 7 + freqOffset);
        // x
        base += encodeSubstr(x + (63 >> freqOffset), 10 + freqOffset, 7 - freqOffset);
        // y 
        base += encodeSubstr(y + (63 >> freqOffset), 17, 7 - freqOffset);
      break;
      case 20:
        // signal
        base += encodeSubstr(signal, 1, 3);
        // x
        base += encodeSubstr(x % 200, 4, 8);
        // y
        base += encodeSubstr(y % 200, 12, 8);
      break;
      case 19:
        // signal
        base += encodeSubstr(signal, 1, 3);
        // data
        base += encodeSubstr(data - 10000, 4, 15);
      break;
      case 17:
        // signal
        base += encodeSubstr(signal, 1, 2);
        // x
        base += encodeSubstr(x + 63, 3, 7);
        // y
        base += encodeSubstr(y + 63, 10, 7);
      break;
      case 11:
        // signal
        base += encodeSubstr(signal, 1, 1);
        // x || y
        base += encodeSubstr(y, 2, 1);
        // data
        base += encodeSubstr(data % 200, 3, 8);
      break;
      case 10:
        base += encodeSubstr(signal, 1, 9);
      break;
      case 9:
        base += encodeSubstr(data, 1, 8);
      break;
      default:
        base = 0;
      break;
    }
    return base;
  }
  int encodeSubstr(int input, int start, int length) {
    return (input << (frequency - start - length));
  }
}
class ECinfo {
  /*
  // class author: Eric Kuebler
  
  * ECinfo builds a registry of every
  ENLIGHTENMENT CENTER on the map

  * an ID of -1 indicates hostile

  * an ID of 0 indicates a neutral

  * only use this class with ECs
  using this class on any other bot
  produces a lot of unnecessary bytecode usage

  * alpha is the EC with the highest ID

  */
  static HashSet<Integer> friendly;
  static ArrayList<ECinfo> ECs;
  static int alpha; // index of command EC
  static int enabledHostiles;
  static void initialize() {
    friendly = new HashSet<Integer>();
    ECs = new ArrayList<ECinfo>();
    alpha = -1;
    enabledHostiles = 0;
  }
  static int update(int ID, int x, int y, int conviction) {
    final int index = ID < 1 ? checkLocation(x, y) : checkID(ID);
    if(index == -1) { // new EC
      ECs.add(new ECinfo(ID, x, y, conviction));
      if(ID > 0) {
        friendly.add(ID);
        if(alpha == -1 || ID > ECs.get(alpha).ID) {
          alpha = ECs.size() - 1;
        } 
      }
      return ECs.size() - 1;
    }
    else {
      final ECinfo get = ECs.get(index);
      if(ID < 1) {
        if(ID != get.ID) {
          if(get.ID > 0) { // EC used to be friendly
            if(!RobotPlayer.rc.canGetFlag(get.ID)) { // check death
              get.ID = ID;
            }
          }
          else { // neutral captured by enemy
            get.ID = ID;
          }
        }
        if(conviction != -1) {
          get.conviction = conviction;
        }
      }
      else { // ID != -1, friendly EC
        if(get.x == -1 && get.y == -1 && x != -1 && y != -1) {
          final int duplicate = checkLocation(x, y);
          if(duplicate != -1) {
            ECs.remove(duplicate);
            if(alpha != -1 && duplicate < alpha) {
              --alpha;
            }
          }
          get.x = x;
          get.y = y;
          get.distance = Path.distance(RobotPlayer.posX - x, RobotPlayer.posY - y);
        }
      }
      return index;
    }
  }
  static int checkID(int ID) {
    final int size = ECs.size();
    for(int i = 0; i < size; ++i) {
      if(ID == ECs.get(i).ID) {
        return i;
      }
    }
    return -1;
  }
  static int checkLocation(int x, int y) {
    final int size = ECs.size();
    for(int i = 0; i < size; ++i) {
      if(x == ECs.get(i).x && y == ECs.get(i).y) {
        return i;
      }
    }
    return -1;
  }
  static int getAlphaIndex() {
    final int size = ECs.size();
    if(size == 0) {
      return -1;
    }
    if(alpha != -1 && alpha < size && RobotPlayer.rc.canGetFlag(ECs.get(alpha).ID)) {
      return alpha;
    }
    else { // decide new alpha
      if(alpha != -1 && alpha < size) {
        ECs.get(alpha).ID = -1;
      }
      final int oldAlpha = alpha;
      alpha = -1;
      for(int i = 0; i < size; ++i) {
        final ECinfo get = ECs.get(i);
        if(get.ID > 0) {
          if(RobotPlayer.rc.canGetFlag(get.ID)) {
            if(alpha == -1 || get.ID > ECs.get(alpha).ID) {
              alpha = i;
            }
          }
          else {
            get.ID = -1;
          }
        }
      }
      return alpha;
    }
  }
  static int getBestHostile() {
    int index = -1; // index of current selection
    int type = -1; // indicates hostile (-1) or neutral (0)
    int dist = -1; // compare distance
    int conviction = -1; // compare conviction
    boolean clogged = false; // only relevant for hostiles
    enabledHostiles = 0;
    int fix = -1; // used for correcting errors in registry
    final int size = ECs.size();
    for(int i = 0; i < size; ++i) {
      if(ECs.get(i).ID < 1) { // neutral or hostile
        final ECinfo get = ECs.get(i);
        if(get.x == -1 || get.y == -1) {
          if(fix == -1) {
            fix = i;
          }
          continue;
        }
        if(get.ID == 0) { // neutral
          if(index == -1 || type == -1 || 
            (get.conviction < conviction && get.distance > dist) || 
            (Math.abs(get.conviction - conviction) < 70 && get.distance < dist)) 
          {
            index = i;
            type = 0;
            dist = get.distance;
            conviction = get.conviction;
          }
        }
        else { // hostile
          if(!get.clogged) {
            ++enabledHostiles;
          }
          if(type != 0) {
            if(index == -1 || (get.distance < dist && (!get.clogged || clogged)) || (clogged && !get.clogged)) {
              index = i;
              type = -1;
              dist = get.distance;
              clogged = get.clogged;
            }
          }
        }
      }
    }
    if(fix != -1) {
      ECs.remove(fix);
    }
    return index;
  }
  static void setClogged(boolean value, int x, int y) {
    final int index = checkLocation(x, y);
    if(index != -1) {
      ECs.get(index).clogged = value;
    }
  }
  int ID;
  int x;
  int y;
  int conviction;
  int distance;
  boolean clogged = false;
  ECinfo(int in_ID, int in_x, int in_y, int in_conviction) {
    ID = in_ID;
    x = in_x;
    y = in_y;
    conviction = in_conviction;
    distance = (x == -1 || y == -1) ? -1 : Path.distance(RobotPlayer.posX - x, RobotPlayer.posY - y);
  }
}
class VoteKit {
  /*
  // Class author: Eric Kuebler
  
  This class handles bidding for ECs

  */
  static int votes; // current vote value
  static int enemyVotes; // possible # of enemy votes
  static int bid; // value of bid
  static boolean voted; // represents whether we bid last round
  static void initialize() {
    votes = RobotPlayer.rc.getTeamVotes();
    bid = 1;
    voted = false;
  }
  static void checkpoint(boolean verifyRound) {
    /*
      This function is called multiple times 
      per RobotPlayer.runEC in order to bypass maxing out
      bytecode costs

      What used to occur is that an EC would only try to vote
      once per every 2-3 rounds due to taking 60000+ bytecode
      to get through one iteration
    */
    if(votes < 751) { // majority not obtained yet
      if(verifyRound) {
        try {
          final int newRound = RobotPlayer.rc.getRoundNum();
          if(newRound != RobotPlayer.round) {
            RobotPlayer.round = newRound;
            RobotPlayer.influence = RobotPlayer.rc.getInfluence();
            handleVotes();
          }
        }
        catch(Exception e) {
          System.out.println("VoteKit.checkpoint Exception?");
        }
      }
      else {
        handleVotes();
      }
    }
  }
  static void handleVotes() {
    try {
      final int newVotes = RobotPlayer.rc.getTeamVotes();
      enemyVotes = RobotPlayer.round - newVotes;
      if(enemyVotes > 300) { // if round greater than 350
        if(voted) { // if put in bid last round
          voted = false;
          if(newVotes > votes) { // bid high enough to obtain votes
            if(enemyVotes < 500 && bid > 1) {
              bid -= 1;
            }
          }
          else if(bid < RobotPlayer.influence / 4) { // bid not high enough to obtain votes
            bid += 3;
          }
        }
        if(bid < RobotPlayer.influence / 4) {
          RobotPlayer.rc.bid(bid);
          voted = true;
        }
        votes = newVotes;
      }
    }
    catch(Exception e) {
      System.out.println("VoteKit.handleVotes Exception");
      e.printStackTrace();
    }
  }
}
public strictfp class RobotPlayer {
  static RobotController rc;
  static final RobotType[] types = {
      RobotType.ENLIGHTENMENT_CENTER,
      RobotType.POLITICIAN,
      RobotType.SLANDERER,
      RobotType.MUCKRAKER
  };
  static final Direction[] directions = {
      Direction.NORTH,
      Direction.NORTHEAST,
      Direction.EAST,
      Direction.SOUTHEAST,
      Direction.SOUTH,
      Direction.SOUTHWEST,
      Direction.WEST,
      Direction.NORTHWEST
  };
  static final int[] slanderCost = {63, 85, 107, 130, 154, 178, 203, 228, 255, 282, 310, 339, 368, 399, 431, 463, 497, 532, 568, 605};
  // ALL spawn vars
  static int turnCount = 0;
  static RobotType type;
  static int id;
  static Team team;
  // ALL dynamic vars
  static int round = 0;
  static MapLocation spawn;
  static int posX;
  static int posY;
  /*
    * posX and posY represent the return value of rc.getLocation()

    * posX and posY are automatically initialized at spawn
    and updated at every movement by the tryMove function
  */
  static HashSet<Integer> allies = new HashSet<Integer>();
  static int task = 0;
  static int wait = 0;
  static MapLocation storedLocation; // location storage
  static double rot = 0;
  static Flag flag = new Flag(-1, 0, 0, 0, 0);
  static int wallRadius = 6;
  static int localSlanderers = 0;
  // EC vars
  static int influence;
  static int alliesIndex = 0;
  static int flagPriority = 0;
  static int slanderCooldown = 0;
  // Robot vars
  static int homeECID = -1; // if not -1, an EC is listening to me
  static MapLocation homeEC;
  static int subtask = 0;
  static int shareIDFlag = 0;
  static boolean order66 = false;
  @SuppressWarnings("unused")
  public static void run(RobotController rc) throws GameActionException {
    RobotPlayer.rc = rc;
    try { // first run
      type = rc.getType();
      id = rc.getID();
      team = rc.getTeam();
      round = rc.getRoundNum();
      spawn = rc.getLocation();
      storedLocation = spawn; // avoid null pointer errors
      posX = spawn.x;
      posY = spawn.y;
      if(type == RobotType.ENLIGHTENMENT_CENTER) { // EC
        VoteKit.initialize();
        ECinfo.initialize();
        ECinfo.update(id, posX, posY, -1);
        if(round > 1) { // THIS is captured hostile EC
          slanderCooldown = 50;
        }
      }
      else {
        Path.initialize();
        final RobotInfo[] nearby = rc.senseNearbyRobots(2, team);
        for(RobotInfo robot : nearby) { // search for local EC
          if(robot.type == RobotType.ENLIGHTENMENT_CENTER) {
            allies.add(robot.ID);
            homeECID = robot.ID;
            homeEC = robot.location;
            storedLocation = robot.location;
            break;
          }
        }
        if(homeECID == -1) { // created elsewhere
          homeEC = spawn;
        }
        MapLocation radius;
        switch(type) {
          case POLITICIAN:
            task = 2;
          break;
          case SLANDERER:
            task = 0;
            radius = radialLocation(homeEC.x, homeEC.y, 3, rng(0, 2 * Math.PI));
            Path.setNewTarget(radius.x, radius.y);
          break;
          case MUCKRAKER:
            task = 0;
            radius = radialLocation(homeEC.x, homeEC.y, 65, (round * 0.3316) % (2 * Math.PI));
            Path.setNewTarget(radius.x, radius.y);
          break;
        }
      }
    }
    catch(Exception e) {
      System.out.println(type + " Run Exception");
      e.printStackTrace();
    }
    while(true) { // execute per round (barring max bytecode costs)
      try {
        round = rc.getRoundNum();
        turnCount += 1;
        if(wait > 0) {
          --wait;
        }
        if(type == RobotType.ENLIGHTENMENT_CENTER) { // EC (Enlightenment Center)
          runEC();
        }
        else { // MB (Mobile Bot)
          runMB();
          Path.handleMovement();
        }
        Clock.yield();
      }
      catch(Exception e) {
        System.out.println(type + " While(true) Exception");
        e.printStackTrace();
      }
    }
  }
  static boolean tryMove(Direction dir) throws GameActionException {
    try {
      if(rc.canMove(dir)) {
        rc.move(dir);
        final MapLocation dirOffset = posDirOffset(dir); // update posX and pos Y
        posX += dirOffset.x;
        posY += dirOffset.y;
        return true;
      }
    }
    catch(Exception e) {
      System.out.println("tryMove Exception");
      e.printStackTrace();
    }
    return false;
  }
  static MapLocation posDirOffset(Direction dir) throws GameActionException {
    int x = 0;
    switch(dir) {
      case NORTHEAST:
      case EAST:
      case SOUTHEAST:
        x = 1;
      break;
      case NORTHWEST:
      case WEST:
      case SOUTHWEST:
        x = -1;
      break;
    }
    int y = 0;
    switch(dir) {
      case NORTHEAST:
      case NORTH:
      case NORTHWEST:
        y = 1;
      break;
      case SOUTHEAST:
      case SOUTH:
      case SOUTHWEST:
        y = -1;
      break;
    }
    return new MapLocation(x, y);
  }
  static void runEC() throws GameActionException {
    influence = rc.getInfluence();
    VoteKit.checkpoint(false); // VOTE CHECKPOINT
    if(!allies.isEmpty()) { // loop friendly MB flags
      int remove = -1;
      int count = 0;
      for(int ID : allies) {
        if(count++ < alliesIndex) { // skips ahead to alliesIndex
          continue;
        }
        if(count > alliesIndex + 30) { // sets maximum ally scans to reduce bytecost
          break;
        }
        if(rc.canGetFlag(ID)) {
          final Flag allyFlag = Flag.decode(rc.getFlag(ID));
          switch(allyFlag.frequency) {
            case 24:
            case 23:
            case 22:
            case 21:
              final int x = allyFlag.x + posX;
              final int y = allyFlag.y + posY;
              switch(allyFlag.signal) {
                case 0: // neutral EC found
                case 1: // hostile EC found
                  ECinfo.update(allyFlag.signal == 0 ? 0 : -1, x, y, allyFlag.data); // neutral or hostile EC found
                  //System.out.println((allyFlag.signal == 0 ? "Neutral" : "Hostile") + " EC info received: conviction: " + allyFlag.data + ", [" + x + ", " + y + "]");
                break;
                case 2: // hostile EC clogged
                  // uses data value to determine clogged or not
                  ECinfo.update(-1, x, y, -1);
                  ECinfo.setClogged(allyFlag.data != 0, x, y);
                break;
              }
            break;
            case 20:
              if(allyFlag.signal == 6) { // order66 survivor found
                setFlag(3, allyFlag);
              }
            break;
            case 19:
              if(allyFlag.signal == 1 && allyFlag.data != id) { // friendly EC ID found
                ECinfo.update(allyFlag.data, -1, -1, -1);
                //System.out.println("friendly reported: " + allyFlag.data);
              }
            break;
            case 11: // edge
              if(allyFlag.signal == 1) { // border is on an edge
                final int newRadius = Math.abs((allyFlag.y == 1 ? posY : posX) - allyFlag.data) + 1;
                if(newRadius > wallRadius) {
                  wallRadius = newRadius;
                }
              }
            break;
          }
        }
        else if(remove != -1) {
          remove = ID;
        }
      }
      if(remove != -1) {
        allies.remove(remove);
      }
      alliesIndex += 30;
      if(alliesIndex > allies.size()) {
        alliesIndex = 0;
      }
    }
    int bestHostile = ECinfo.getBestHostile();
    if(round > 800 && ECinfo.enabledHostiles == 0) {
      task = 66;
    }
    VoteKit.checkpoint(true); // VOTE CHECKPOINT
    final int alphaID = ECinfo.ECs.get(ECinfo.getAlphaIndex()).ID;
    /*if(round % 5 == 0) {
      System.out.println("task: " + task + "\nalphaID: " + alphaID);
    }*/
    if(ECinfo.friendly.size() > 1) {
      int remove = -1;
      try {
        for(Integer ID : ECinfo.friendly) {
          if(ID != id) {
            if(rc.canGetFlag(ID)) {
              final int encoded = rc.getFlag(ID);
              if(encoded != 0) {
                final Flag getFlag = Flag.decode(encoded);
                switch(getFlag.frequency) {
                  case 20:
                    switch(getFlag.signal) {
                      case 0:
                        final int index = ECinfo.checkID(ID);
                        if(index != -1) {
                          final ECinfo get = ECinfo.ECs.get(index);
                          if(get.x == -1 || get.y == -1) {
                            ECinfo.update(ID, getFlag.x, getFlag.y, -1);
                            setFlag(20, new Flag(19, 1, ID, -1, -1));
                            //System.out.println("friendly location: " + getFlag.x+ ", " + getFlag.y);
                          }
                        }
                      break;
                      case 3:
                        if(task != 1) {
                          bestHostile = ECinfo.update(getFlag.data == 3 ? -1 : 0, getFlag.x, getFlag.y, -1);
                          task = 1;
                          setFlag(5, getFlag); // relay Alpha attack statement
                        }
                      break;
                      case 6:
                        if(bestHostile == -1) {
                          System.out.println("Echoing order66 on: [" + getFlag.x + ", " + getFlag.y + "]");
                          setFlag(3, getFlag);
                          
                        }
                      break;
                    }            
                  break;
                  case 19:
                    if(getFlag.signal == 1 && getFlag.data != id) { // friendly EC ID found
                      ECinfo.update(getFlag.data, -1, -1, -1);
                    }
                  break;
                  case 10:
                    if(alphaID == ID) {
                      setFlag(2, getFlag); // relay Alpha task update statement
                    }
                  break;
                }
              }
            }
            else {
              if(remove == -1) {
                remove = ID;
              }
            }
          }
        }
        if(remove != -1) {
          ECinfo.friendly.remove(remove);
          final int index = ECinfo.checkID(remove);
          if(index != -1) {
            ECinfo.ECs.get(index).ID = -1; // fix registry
          }
        }
      }
      catch(Exception e) {
        System.out.println("ECinfo.friendly for loop exception");
      }
    }
    int threat = 0;
    int localPoliticians = 0;
    localSlanderers = 0;
    int localMuckrakers = 0;
    if(rc.isReady()) { // reduce bytecode use significantly
      final RobotInfo[] nearby = rc.senseNearbyRobots(-1);
      for(RobotInfo robot : nearby) { // scan local robots
        if(robot.team != team) {
          if(slanderCooldown < 100) {
            slanderCooldown += 3;
          }
          if(robot.type == RobotType.ENLIGHTENMENT_CENTER) {
            ECinfo.update(robot.team == Team.NEUTRAL ? 0 : -1, robot.location.x, robot.location.y, robot.conviction);
          }
          else if(robot.conviction > threat) {
            threat += robot.conviction;
          }
        }
        else if(!allies.contains(robot.ID)) { // not an ally, check flag
          final Flag nearFlag = Flag.decode(rc.getFlag(robot.ID));
          switch(nearFlag.frequency) {
            case 19:
              if(nearFlag.signal == 1) { // friendly EC ID found
                ECinfo.update(nearFlag.data, -1, -1, -1);
              }
            break;
          }
        }
        else switch(robot.type) {
          case POLITICIAN:
            if(robot.conviction < 50) { // ensure defense only is counted
              ++localPoliticians;
            }
          break;
          case SLANDERER:
            ++localSlanderers;
          break;
          case MUCKRAKER:
            ++localMuckrakers;
          break;
        }
      }
      if((threat != 0 && localPoliticians < 5) || localPoliticians < 3) {
        if(threat > influence) {
          buildRobot(RobotType.POLITICIAN, 19);
        }
        else {
          buildRobot(RobotType.POLITICIAN, (int)Math.max(threat + 10, task == 66 ? 30 : 19));
        }
      }
      else if(slanderCooldown != 0) {
        --slanderCooldown;
      }
      else if(localSlanderers < (task == 66 ? 15 : 10)) {
        if(buildBestSlanderer((int)(influence / 3))) {
          slanderCooldown = 10;
        }
      }
    }
    if(bestHostile >= ECinfo.ECs.size()) {
      bestHostile = -1;
    }
    switch(task) { // ALL MODES ARE DEFENSE MODE
      case 0: // scout mode
        if(threat == 0) {
          buildRobot(RobotType.MUCKRAKER, 1);
        }
        if(bestHostile != -1) {
          final ECinfo get = ECinfo.ECs.get(bestHostile);
          if(get.x != -1 && get.y != -1 && !get.clogged) {
            task = 1;
          }
        }
      break;
      case 1: // assault mode
        if(bestHostile == -1) {
          task = 0;
        }
        else {
          final ECinfo get = ECinfo.ECs.get(bestHostile);
          setFlag(alphaID == id ? 5 : 4, new Flag(20, get.ID == 0 || get.
          clogged ? 4 : 3, get.x, get.y, -1));
          System.out.println("Sending army: " + get.x + ", " + get.y);
        }
        if(threat == 0 && ++subtask > 2) {
          subtask = 0;
          buildRobot(RobotType.MUCKRAKER, 1);
        }
        else {
          // spawn troop at 2/3 the cost of hostile EC
          buildRobot(RobotType.POLITICIAN, (int)Math.max(50, ((ECinfo.ECs.get(bestHostile).conviction << 1) / 3) + 11));
        }
      break;
      case 66: // order 66
        setFlag(2, new Flag(10, 66, -1, -1, -1)); // execute order 66
        if(++subtask > 50) {
          buildRobot(RobotType.MUCKRAKER, 1);
          subtask = 0;
        }
      break;
      case 2016: // wall mode
        buildRobot(RobotType.MUCKRAKER, 1);
        setFlag(8, new Flag(9, 0, wallRadius, -1, -1));
      break;
    }
    // set flag
    if(turnCount % 3 == 0 || flagPriority > 9) { // every third turn
      if(flagPriority == 0) {
        rc.setFlag(0);
      }
      else {
        rc.setFlag(flag.encode());
        if(flagPriority > 9) {
          flagPriority = 9; // make priority last 2 rounds
        }
        else {
          flagPriority = 0;
        }
      }
    }
    else { // other rounds
      rc.setFlag(new Flag(20, 0, posX, posY, -1).encode()); // encode our own location
    }
    VoteKit.checkpoint(true); // VOTE CHECKPOINT
  }
  static boolean buildRobot(RobotType type, int influenceSpend) throws GameActionException {
    if(influence >= influenceSpend && rc.isReady()) {
      for(Direction dir : directions) {
        if(rc.canBuildRobot(type, dir, influenceSpend)) {
          rc.buildRobot(type, dir, influenceSpend);
          final MapLocation dirOffset = posDirOffset(dir);
          try {
            final RobotInfo robot = rc.senseRobotAtLocation(new MapLocation(posX + dirOffset.x, posY + dirOffset.y));
            allies.add(robot.ID);
          }
          catch(Exception e) {
            System.out.println("add ally ID Exception");
            e.printStackTrace();
          }
          return true;
        }
      }
      // if code reaches this segment
      // base is surrounded
      // check if surrounded by allies or hostiles
      final int surroundingHostiles = rc.senseNearbyRobots(2, team == Team.A ? Team.B : Team.A).length;
      if(surroundingHostiles == 0) {
        setFlag(10, new Flag(10, 32, -1, -1, -1));
      }
      else {
        // hostiles surrounding
      }
    }
    return false;
  }
  static boolean buildBestSlanderer(int investment) throws GameActionException {
    if(investment < 85) {
      if(buildRobot(RobotType.SLANDERER, 63)) {
        return true;
      }
    }
    else if(investment > 604) {
      if(buildRobot(RobotType.SLANDERER, 605)) {
        return true;
      }
    }
    else {
      for(int i = 2; i < 20; ++i) {
        if(investment < slanderCost[i]) {
          if(buildRobot(RobotType.SLANDERER, slanderCost[i - 1])) {
            return true;
          }
          else {
            break;
          }
        }
      }
    }
    return false;
  }
  static void runMB() throws GameActionException {
    // logical combination of all mobile robots to
    // maintain consistency and reduce redundency
    final double empowerFactor = rc.getEmpowerFactor(team, 0);
    final int conviction = (int)Math.floor(rc.getConviction() * empowerFactor);
    if(type == RobotType.SLANDERER && turnCount > 300) { // type conversion
      type = RobotType.POLITICIAN;
    }
    int empowerDist = 0;
    int empowerCount = 0;
    int newLocalSlanderers = type == RobotType.SLANDERER ? 1 : 0;
    final RobotInfo[] nearby = rc.senseNearbyRobots(-1);
    for(RobotInfo robot : nearby) { // scan local robots
      if(robot.team == team) {
        if(robot.type == RobotType.ENLIGHTENMENT_CENTER) {
          if(!allies.contains(robot.ID)) {
            allies.add(robot.ID);
            setFlag(20, new Flag(19, 1, robot.ID, -1, -1));
          }
          shareIDFlag = new Flag(19, 1, robot.ID, -1, -1).encode();
          switch(type) {
            case POLITICIAN:
              if(Path.atTarget(robot.location.x, robot.location.y)) {
                Path.done = true;
                setFlag(20, new Flag(19, 1, robot.ID, -1, -1));
              }
            break;
            case SLANDERER:
              if((Path.done || Path.stuck) && Path.distance(posX - robot.location.x, posY - robot.location.y) <= 4) {
                Path.setNewTarget((posX << 1) - robot.location.x, (posY << 1) - robot.location.y);
              }
            break;
            case MUCKRAKER:
              if((task == 10 || task == 11) && storedLocation.x == robot.location.x && storedLocation.y == robot.location.y) {
                task = 0;
                Path.done = true;
              }
            break;
          }
        }
        else if(robot.type == RobotType.SLANDERER) {
          ++newLocalSlanderers;
        }
        else if(type == RobotType.POLITICIAN && robot.type == RobotType.POLITICIAN) {
          for(int i = 0; i < 20; ++i) {
            if(robot.influence == slanderCost[i]) {
              ++newLocalSlanderers;
              break;
            }
          }
        }
        if(rc.canGetFlag(robot.ID)) {
          final Flag allyFlag = Flag.decode(rc.getFlag(robot.ID));
          switch(allyFlag.frequency) {
            case 19: //shareID protocol
              if(allyFlag.data != homeECID) {
                if(allies.add(allyFlag.data) || shareIDFlag == 0) {
                  shareIDFlag = new Flag(19, 1, allyFlag.data, -1, -1).encode();
                }
              }
            break;
            case 17:
              // relay local threat message
              final int x = allyFlag.x + robot.location.x;
              final int y = allyFlag.y + robot.location.y;
              if(allyFlag.signal - 1 != -1 && (flag.frequency != 17 || allyFlag.signal > flag.signal)) {
                setFlag(2, new Flag(17, allyFlag.signal - 1, x - posX, y - posY, -1));
              }
              if(type == RobotType.POLITICIAN && (order66 || (task != 1 && Path.distance(homeEC.x - x, homeEC.y - y) <= 100))) {
                if(!Path.atTarget(x, y)) {
                  if(order66) { // eliminate
                    task = 0;
                    Path.setNewTarget(x, y);
                  }
                  else { // defend
                    task = 4;
                    Path.setNewTarget(x, y);
                  }
                }
              }
            break;
            case 10:
              if(allyFlag.signal == 32 && task != 10 && task != 11 && task != 2016 && task != 2020) { // spread protocol
                if(Path.stuckCount > 10) {
                  setFlag(20, new Flag(10, 32, -1, -1, -1));
                }
                else if(type != RobotType.SLANDERER || !Path.edge || Path.distance(posX - robot.location.x, posY - robot.location.y) < 9) {
                  // if statement here makes SLANDERER's cling to the edge if not in the way
                  task = 0;
                  Path.setNewTarget((posX << 1) - robot.location.x, (posY << 1) - robot.location.y);
                }
              }
            break;
          }
        }
      }
      else { // hostile target
        if(order66 && homeECID != -1) {
          setFlag(3, new Flag(20, 6, robot.location.x, robot.location.y, -1));
        }
        else if(order66 || (robot.type != RobotType.ENLIGHTENMENT_CENTER && (robot.type != RobotType.MUCKRAKER || localSlanderers != 0))) {
          setFlag(2, new Flag(17, 3, robot.location.x - posX, robot.location.y - posY, -1));
        }
        if(robot.type == RobotType.ENLIGHTENMENT_CENTER) { // hostile EC
          // ping location to homeEC
          if(homeECID != -1) {
            setFlag(9, new Flag(24, robot.team == Team.NEUTRAL ? 0 : 1, robot.conviction, robot.location.x - homeEC.x, robot.location.y - homeEC.y));
          }
          switch(type) { // reassign task
            case POLITICIAN:
              if(order66 || (task == 1 && Path.atTarget(robot.location.x, robot.location.y)) || conviction > robot.conviction) {
                final int dist = Path.distance(posX - robot.location.x, posY - robot.location.y);
                if(dist <= 9) {
                  if(rc.canEmpower(dist)) {
                    if(dist > 2 && !Path.stuck) {
                      // make sure with range it will convert still
                      int count = 0;
                      for(RobotInfo robotDivide : nearby) {
                        if(robotDivide.type != RobotType.ENLIGHTENMENT_CENTER) {
                          if(Path.distance(posX - robotDivide.location.x, posY - robotDivide.location.y) <= dist) {
                            ++count;
                          }
                        }
                      }
                      if(count == 0 || (conviction - 10) / count > robot.conviction) {
                        rc.empower(dist);
                      }
                    }
                    else {
                      rc.empower(dist);
                    }
                  }
                }
              }
            break;
            case MUCKRAKER:
              if(task == 0 && robot.team != Team.NEUTRAL && wait == 0) {
                wait = 5;
                final MapLocation clog = clogLocation(robot.location, false);
                if(clog != null) {
                  if(clog.x == -1) {
                    if(homeECID != -1) {
                      setFlag(10, new Flag(24, 2, 1000, robot.location.x - homeEC.x, robot.location.y - homeEC.y)); // report clogged
                    }
                  }
                  else {
                    storedLocation = robot.location;
                    task = 10;
                    Path.setNewTarget(clog.x, clog.y);
                  }
                }
              }
            break;
          }
        }
        else switch(type) { // switch THIS RobotType for hostiles
          case POLITICIAN:
            final int homeDist = Path.distance(robot.location.x - homeEC.x, robot.location.y - homeEC.y);
            if(
              task != 1 && (
              order66 || 
              (robot.type == RobotType.MUCKRAKER && localSlanderers != 0) || 
              (robot.type != RobotType.MUCKRAKER && compareConviction(conviction, robot.conviction)) ||
               homeDist <= 4
              )) {
              final int dist = Path.distance(posX - robot.location.x, posY - robot.location.y);
              if(dist <= 9 && rc.canEmpower(dist) && dist > empowerDist) {
                empowerDist = dist;
                empowerCount += robot.type == RobotType.MUCKRAKER && homeDist > 4 ? 1 : 3;
              }
              else if(task != 4 || dist < Path.targetDistance) {
                task = 4;
                Path.setNewTarget(robot.location.x, robot.location.y);
              }
            }
          break;
          case SLANDERER:
            final int tarX = (posX << 1) - robot.location.x;
            final int tarY = (posY << 1) - robot.location.y;
            if(Path.done || !Path.atTarget(tarX, tarY)) {
              Path.setNewTarget(tarX, tarY);
            }
          break;
          case MUCKRAKER:
            if(robot.type == RobotType.SLANDERER) {
              if(rc.canExpose(robot.location)) {
                rc.expose(robot.location); // destroy slanderer
              }
              else if(task == 0 && !Path.atTarget(robot.location.x, robot.location.y)) { // hunt slanderer
                Path.setNewTarget(robot.location.x, robot.location.y);
              }
            }
            if(task == 2020) {
              subtask = 1; // rigid wall
            }
          break;
        }
      } 
    }
    localSlanderers = newLocalSlanderers;
    /*if(round % 5 == 0 && type == RobotType.POLITICIAN) {
      System.out.println("task: " + task + "\nready: " + rc.isReady() + "\norder66: " + order66 + "\nempowerCount: " + empowerCount + "\nempowerDist: " + empowerDist);
    }*/
    if(type == RobotType.POLITICIAN && empowerDist != 0 && (localSlanderers != 0 || order66 || empowerCount > 2 || Path.stuck)) {
      rc.empower(empowerDist);
    }
    if(homeECID != -1) {
      if(rc.canGetFlag(homeECID)) {
        final int encoded = rc.getFlag(homeECID);
        if(encoded != 0) {
          final Flag flagEC = Flag.decode(encoded);
          switch(flagEC.frequency) {
            case 20:
              switch(flagEC.signal) {
                case 3: // assault on hostile target
                  if(!Path.atTarget(flagEC.x, flagEC.y)) {
                    switch(type) {
                      case POLITICIAN:
                        if(task != 4 && conviction > 30 && (task != 1 || !Path.atTarget(flagEC.x, flagEC.y))) {
                          task = 1;
                          Path.setNewTarget(flagEC.x, flagEC.y);
                        }
                      break;
                      case SLANDERER: // move opposite direction
                        if(!Path.edge && (Path.done || Path.stuck)) {
                          Path.setNewTarget((posX << 1) - flagEC.x, (posY << 1) - flagEC.y);
                        }
                      break;
                      case MUCKRAKER:
                        if(task == 0 && (storedLocation.x != flagEC.x || storedLocation.y != flagEC.y) && Path.distance(posX - flagEC.x, posY - flagEC.y) < 200) {
                          storedLocation = new MapLocation(flagEC.x, flagEC.y);
                          Path.setNewTarget(flagEC.x, flagEC.y);
                        }
                      break;
                    }
                  }
                  if(order66) {
                    order66 = false;
                  }
                break;
                case 4: // assault on neutral target (or clogged hostile)
                  if(type == RobotType.POLITICIAN && task != 4 && conviction > 30 && (task != 1 || !Path.atTarget(flagEC.x, flagEC.y))) {
                    task = 1;
                    Path.setNewTarget(flagEC.x, flagEC.y);
                  }
                  if(order66) {
                    order66 = false;
                  }
                break;
                case 6: // hunt order 66 survivor
                  order66 = true;
                  if(type == RobotType.POLITICIAN) {
                    if(order66 && (task != 0 || Path.distance(posX - flagEC.x, posY - flagEC.y) < Path.distance(posX - Path.target.x, posY - Path.target.y))) {
                      task = 0;
                      Path.setNewTarget(flagEC.x, flagEC.y);
                    }
                  }
                break;
              }
            break;
            case 10:
              if(flagEC.signal == 66) { // order 66
                order66 = true;
              }
            break;
            case 9:
              if(wallRadius != flagEC.data) {
                wallRadius = flagEC.data;
                if(task == 2021) {
                  task = 2020;
                }
              }
            break;
          }
        }
      }
      else { // homeEC captured
        allies.remove(homeECID);
        homeECID = -1;
        if(!allies.isEmpty()) {
          for(int ID : allies) {
            homeECID = ID;
            break;
          }
        }
      }
    }
    switch(task) {
      case 0: // go target, no specific objective
      case 4: // on defense, same as 0 except will not set to 1
        if(Path.done || Path.edge || Path.stuck) {
          switch(type) {
            case POLITICIAN:
              task = 2; // sentry rotation
              storedLocation = new MapLocation(posX, posY);
            break;
            case SLANDERER:
              if(Path.stuck && !Path.edge && wait == 0) { // stay on edge
                Path.setNewTarget(Path.target.x, Path.target.y);
                wait = 30;
              }
            break;
            case MUCKRAKER:
              rot = (rot + Math.PI / 3) % (2 * Math.PI);
              final MapLocation radius = radialLocation(posX, posY, 65, rot);
              Path.setNewTarget(radius.x, radius.y); // go somewhere new
            break;
          }
        }
      break;
      case 1: // go target, attack EC ONLY
        if(Path.stuck || Path.done) {
          task = 2;
          storedLocation = radialLocation(posX, posY, 6, rng(0, 2 * Math.PI));
        }
      break;
      case 2: // sentry around homeEC
        if(Path.done || Path.stuck) {
          rot = (rot + 0.4) % (2 * Math.PI);
          final MapLocation radius = radialLocation(homeEC.x, homeEC.y, wallRadius, rot);
          Path.setNewTarget(radius.x, radius.y);
        }
      break;
      case 10: // outer surrounding circle on hostile EC
        if(Path.distance(posX - storedLocation.x, posY - storedLocation.y) <= 2) {
          Path.done = true;
          wait = 50;
          task = 11; // in position
        }
        else if(wait == 0) {
          wait = 5;
          final MapLocation clog = clogLocation(storedLocation, false);
          if(clog != null) {
            if(homeECID != -1) { // report cloggage
              setFlag(10, new Flag(24, 2, clog.x == -1 ? 1000 : 0, storedLocation.x - homeEC.x, storedLocation.y - homeEC.y)); // report clogged
            }
            if(clog.x == -1) {
              task = 0;
              final MapLocation radius = radialLocation(spawn.x, spawn.y, 65, rot);
              Path.setNewTarget(radius.x, radius.y);
            }
            else {
              Path.setNewTarget(clog.x, clog.y);
            }
          }
        }
      break;
      case 11: // inner surrounding circle on hostile EC
        if(wait == 0) {
          wait = 50;
          final MapLocation clog = clogLocation(storedLocation, true);
          if(clog != null) {
            if(homeECID != -1) { // report cloggage
              setFlag(10, new Flag(24, 2, clog.x == -1 ? 1000 : 0, storedLocation.x - homeEC.x, storedLocation.y - homeEC.y)); // report clogged
            }
            if(clog.x != -1 && Path.distance(posX - clog.x, posY - clog.y) <= 2) { // error correction in strange circumstances
              Path.setNewTarget(clog.x, clog.y);
            }
          }
        }
      break;
      case 2016: // wall protocol
        if(Path.done || Path.stuck) { // next rotation
          if(Path.done && storedLocation != null && posX == storedLocation.x && posY == storedLocation.y) {
            task = 2020;
            subtask = 0;
            final int offsetX = Math.abs(posX - homeEC.x) - wallRadius;
            final int offsetY = Math.abs(posY - homeEC.y) - wallRadius;
            final boolean onX = offsetX == 0 || offsetX == 1;
            final boolean onY = offsetY == 0 || offsetY == 1;
            final MapLocation location = new MapLocation(
              posX + (onX ? (posX < homeEC.x ? -1 : 1) : 0),
              posY + (onY ? (posY < homeEC.y ? -1 : 1) : 0)
            );
            if(!rc.onTheMap(location)) { // smart wall expansion
              setFlag(3, new Flag(11, 1, storedLocation.x == 0 ? 1 : 0, storedLocation.x == 0 ? posY : posX, -1));
            }
          }
          else if(++subtask == 5) {
            task = 0; // wall secure, time to scout
            final MapLocation radius = radialLocation(homeEC.x, homeEC.y, 65, rng(0, 2 * Math.PI));
            Path.setNewTarget(radius.x, radius.y);
          }
          else {
            Path.setNewTarget(homeEC.x + (subtask < 3 ? wallRadius - 1 : 1 - wallRadius), homeEC.y + ((subtask == 4 || subtask == 1) ? wallRadius - 1 : 1 - wallRadius));
          }
        }
        else if(wait == 0) { // fill in empty positions
          wait = 5;
          storedLocation = reinforceWall(posX, posY);
          if(storedLocation != null) {
            Path.setNewTarget(storedLocation.x, storedLocation.y);
          }
        }
      break;
      case 2020: // in wall position
        if(subtask == 1) { // threat nearby
          if(posX == storedLocation.x && posY == storedLocation.y) {
            if(Math.abs(posX - homeEC.x) == wallRadius + 1) {
              Path.setNewTarget(posX + (posX < homeEC.x ? 1 : -1), posY);
            }
            else if(Math.abs(posY - homeEC.y) == wallRadius + 1) {
              Path.setNewTarget(posX, posY + (posY < homeEC.y ? 1 : -1));
            }
          }
        }
        else if(posX != storedLocation.x || posY != storedLocation.y) {
          task = 2016;
          Path.setNewTarget(storedLocation.x, storedLocation.y);
        }
        subtask = 0;
      break;
    }
    // set flag
    if(flagPriority != 0) {
      rc.setFlag(flag.encode());
      flagPriority = 0;
    }
    else {
      flag = new Flag(-1, 0, 0, 0, 0);
      if(homeECID != -1) {
        if(turnCount % 2 == 0) {
          rc.setFlag(new Flag(19, 0, homeECID, -1, -1).encode());
        }
        else if(shareIDFlag != 0) {
          rc.setFlag(shareIDFlag);
          shareIDFlag = 0;
        }
      }
    }
  }
  static boolean setFlag(int priority, Flag inputFlag) {
    if(inputFlag.frequency == -1) {
      return false;
    }
    if(priority > flagPriority) {
      flagPriority = priority;
      flag = inputFlag;
      return true;
    }
    return false;
  }
  static boolean compareConviction(int conviction, int enemyConviction) throws GameActionException {
    return conviction > enemyConviction && !(conviction > 49 && enemyConviction < 10);
  }
  static MapLocation clogLocation(MapLocation center, boolean innerOnly) throws GameActionException {
    // potentially very expensive function
    if(!rc.canSenseLocation(center)) { // cost 5
      return null;
    }
    try {
      // check immediate surrounding
      boolean hostile = false;
      for(int dy = -1; dy < 2; ++dy) { 
        for(int dx = -1; dx < 2; ++dx) {
          if(dy == 0 && dx == 0) {
            continue;
          }
          final MapLocation location = new MapLocation(center.x + dx, center.y + dy);
          if(rc.canSenseLocation(location)) { // cost 5
            final RobotInfo robot = rc.senseRobotAtLocation(location); // cost 25
            if(robot == null) {
              return location;
            }
            else if(robot.team != team) {
              hostile = true;
            }
          }
        }
      }
      if(!hostile || innerOnly) {
        return new MapLocation(-1, -1);
      }
      // check surrounding surrounding
      for(int dy = -2; dy < 3; ++dy) { 
        for(int dx = -2; dx < 3; ++dx) {
          if(dy == -2 || dy == 2 || dx == -2 || dx == 2) {
            final MapLocation location = new MapLocation(center.x + dx, center.y + dy);
            if(rc.canSenseLocation(location)) { // cost 5
              final RobotInfo robot = rc.senseRobotAtLocation(location); // cost 25
              if(robot == null || robot.team != team) {
                return location;
              }
            }
          }
        }
      }
      return new MapLocation(-1, -1);
    }
    catch(Exception e) {
      System.out.println("surroundingLocation exception");
    }
    return null;
  }
  static MapLocation reinforceWall(int x, int y) {
    try {
      for(int dy = -2; dy < 3; ++dy) {
        for(int dx = -2; dx < 3; ++dx) {
          final MapLocation location = new MapLocation(x + dx, y + dy);
          final int offsetX = Math.abs(location.x - homeEC.x);
          final int offsetY = Math.abs(location.y - homeEC.y);
          final boolean diagonal = (offsetX + offsetY) % 2 == 0;
          if(diagonal &&
            (
              ((offsetX == wallRadius || offsetX == wallRadius + 1) && offsetY <= wallRadius) || 
              ((offsetY == wallRadius || offsetY == wallRadius + 1) && offsetX <= wallRadius)
            )
          ) { // on the border
            if(rc.canSenseLocation(location)) { // cost 5
              final RobotInfo robot = rc.senseRobotAtLocation(location); // cost 25
              if(robot == null) { // empty map location
                return location;
              }
            }
          }
        }
      }
      return null;
    }
    catch(Exception e) {
      System.out.println("reinforceWall Exception");
    }
    return null;
  }
  static MapLocation fixedLocation(int x, int y) throws GameActionException {
    /*
      this function allows robots
      to share coordinates without
      a shared reference position by
      generating a common reference position
      manually on each calculation
    */
    int fixedX = posX - (posX % 200) + x;
    if(fixedX < posX - 64) {
      fixedX += 200;
    }
    else if(fixedX > posX + 64) {
      fixedX -= 200;
    }
    int fixedY = posY - (posY % 200) + y;
    if(fixedY < posY - 64) {
      fixedY += 200;
    }
    else if(fixedY > posY + 64) {
      fixedY -= 200;
    }
    return new MapLocation(fixedX, fixedY);
  }
  // mathematical functions
  static MapLocation radialLocation(int x, int y, int radius, double theta) throws GameActionException {
    return new MapLocation(
      x + (int)(radius * Math.cos(theta)),
      y + (int)(radius * Math.sin(theta))
    );
  }
  static double rng(double min, double max) throws GameActionException {
    return Math.random() * (max - min + 1) + min;
  }
}