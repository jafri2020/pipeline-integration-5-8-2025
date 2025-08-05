import requests
from pprint import pformat
from langchain_core.tools import Tool
from langchain_openai import ChatOpenAI
from langchain_core.messages import HumanMessage, SystemMessage
from langgraph.graph import MessagesState, START, StateGraph
from langgraph.prebuilt import tools_condition, ToolNode
from langgraph.checkpoint.memory import MemorySaver
from database import get_data
from pydantic import BaseModel
from dotenv import load_dotenv
from langgraph.checkpoint.sqlite import SqliteSaver
import os, sqlite3
from datetime import datetime
import pytz
from langchain_core.tools import StructuredTool
import time
from langchain_core.messages import ToolMessage


# Load environment variables from .env file
load_dotenv()

username="jane_thompson"

# Load personalized data

# pip install langgraph-checkpoint-sqlite
conn = sqlite3.connect('chatbot_memory.sqlite', check_same_thread=False)
 
# -------------------- Weather & News Tools --------------------
 
def fetch_weather(city: str, api_key: str) -> str:
    url = f"http://api.openweathermap.org/data/2.5/weather?q={city}&appid={api_key}&units=metric"
    try:
        response = requests.get(url)
        data = response.json()
        if response.status_code == 200:
            weather = data['weather'][0]['description']
            temperature = data['main']['temp']
            return f"The current weather in {city} is {weather} with a temperature of {temperature}°C."
        else:
            return f"Error: {data.get('message', 'Failed to fetch weather')}"
    except Exception as e:
        return f"Error: {str(e)}"
 
def fetch_news(topic: str, api_key: str) -> str:
    url = f"https://newsapi.org/v2/everything?q={topic}&apiKey={api_key}"
    try:
        response = requests.get(url)
        data = response.json()
        if response.status_code == 200:
            articles = data.get('articles', [])
            if not articles:
                return f"No news found for {topic}."
            headlines = [f"{i+1}. {article['title']}" for i, article in enumerate(articles[:5])]
            return f"Top news for '{topic}':\n" + "\n".join(headlines)
        else:
            return f"Error: {data.get('message', 'Failed to fetch news')}"
    except Exception as e:
        return f"Error: {str(e)}"
 
# -------------------- Gait and Activity Tools --------------------
 
 
def fetch_gait_posture_data(api_base: str = "http://localhost:8002") -> str:
    """
    Fetches gait posture data blocks from /gait_data_ endpoint.
    Each block should include:
    Start Time, End Time, Gait Speed, Step Length, Step Width, Cadence (Steps/min)
    """
    try:
        response = requests.get(f"{api_base}/gait_data")
        if response.status_code == 200:
            data = response.json()
            if not data:
                return "No gait posture data available."
 
            formatted = "\n".join(
                [
                    f"{i+1}. ⏱ {block.get('Start Time', 'N/A')} → {block.get('End Time', 'N/A')}\n"
                    f"   ➤ Gait Speed: {block.get('Gait Speed', 'N/A')}\n"
                    f"   ➤ Step Length: {block.get('Step Length', 'N/A')}\n"
                    f"   ➤ Step Width: {block.get('Step Width', 'N/A')}\n"
                    f"   ➤ Cadence: {block.get('Cadence (Steps/min)', 'N/A')} Steps/min"
                    for i, block in enumerate(data)
                ]
            )
            return f"📉 Gait Posture Data (Time Blocks):\n{formatted}"
        return "⚠ Failed to fetch gait posture data."
    except Exception as e:
        return f"❌ Error occurred: {str(e)}"
 
 
 
def fetch_activity_data(api_base: str = "http://localhost:8002") -> str:
    try:
        response = requests.get(f"{api_base}/activities/all")
        if response.status_code == 200:
            data = response.json()
            if not data:
                return "No activity data available."
            formatted = "\n".join(
                [f"{i+1}. {block['Activity']} in {block['Place']} "
                 f"(Start: {block['Start_Time']} → End: {block['End_Time']})"
                 for i, block in enumerate(data)]
            )
            return f"🏃 Daily Life Activity Blocks:\n{formatted}"
        return "⚠ Failed to fetch activity data."
    except Exception as e:
        return f"❌ Error occurred: {str(e)}"
 
def fetch_last_activity_data(api_base: str = "http://localhost:8002") -> str:
    try:
        response = requests.get(f"{api_base}/activities/all")
        if response.status_code == 200:
            data = response.json()
            if not data:
                return "No activity data available."
            last = data[-1]
            return f"🕘 Your last activity was '{last['Activity']}' in {last['Place']} from {last['Start_Time']} to {last['End_Time']}."
        return "⚠ Failed to fetch activity data."
    except Exception as e:
        return f"❌ Error occurred: {str(e)}"
 

# -------------------- Tools Binding --------------------
 
class EmptyInput(BaseModel):
    dummy: str = ""
 
weather_api_key = os.getenv("WEATHER_API_KEY")
news_api_key = os.getenv("NEWS_API_KEY")
     # Replace with your NewsAPI key
 
# Tool Functions
def weather_tool_func(city: str) -> str:
    print("[Tool HIT] Weather tool invoked.")
    return fetch_weather(city, api_key=weather_api_key)
 
def news_tool_func(topic: str) -> str:
    print("[Tool HIT] News tool invoked.")
    return fetch_news(topic, api_key=news_api_key)
 
def gait_tool_func(dummy: str = "") -> str:
    print("[Tool HIT] Gait posture tool invoked.")
    return fetch_gait_posture_data()
 
def activity_tool_func(dummy: str = "") -> str:
    print("[Tool HIT] Activity data tool invoked.")
    return fetch_activity_data()
 

from langchain_core.output_parsers import StrOutputParser
from langchain_core.prompts import ChatPromptTemplate

# Create a filtering prompt so the LLM chooses only the relevant parts of the user data
profile_filter_prompt = ChatPromptTemplate.from_messages([
    ("system", "You are Rui, a gentle AI assistant for dementia care. You have access to the user's full profile."),
    ("human", "User profile:\n\n{profile}\n\nQuestion: {query}\n\nReply briefly and directly, based only on what's relevant to the question.")
])

def answer_based_on_userdata(query: str) -> str:
    print("[Tool HIT] User Information tool invoked.")
    print("getting data for", username)
    profile = get_data(username)

    # Feed profile and query into LLM to trim irrelevant info
    chain = profile_filter_prompt | simple_llm | StrOutputParser()
    result = chain.invoke({"profile": profile, "query": query})
    return result.strip()

# def answer_based_on_userdata(query: str) -> str:
#     print("[Tool HIT] User Information tool invoked.")
#     print("getting data for ",username)
#     converted_string = get_data(username)
#     # print(converted_string)
#     print("getting data for ",username)
#     return converted_string
 
 
def fetch_current_datetime(dummy: str = "") -> str:
    print("Fetching current datetime...")  # Debugging line
    timezone = pytz.timezone("Asia/Karachi") # You can change to any timezone (e.g., "UTC, AND more")
    now = datetime.now(timezone)
    return now.strftime("Date: %A, %B %d, %Y\nTime: %I:%M:%S %p")
 
# Define Tools
weather_tool = Tool.from_function(
    func=weather_tool_func,
    name="fetch_weather",
    description="Fetches current weather for a city. Input: city name."
)
 
news_tool = Tool.from_function(
    func=news_tool_func,
    name="fetch_news",
    description="Fetches top news headlines for a topic. Input: topic name."
)
 
gait_posture_tool = Tool(
    name="fetch_gait_posture_data",
    func=gait_tool_func,
    description="Fetches overall gait posture data for user.",
    args_schema=EmptyInput
)
 
daily_activity_tool = Tool(
    name="fetch_activity_data",
    func=activity_tool_func,
    description="Fetches today's daily activity log. Have to share activities when ever asking for activity query.",
    args_schema=EmptyInput
)
 

 
knowledge_tool = Tool(
    name="personalized_user_data_QA",
    func=answer_based_on_userdata,
    description="Use this tool to answer any question about user's personal profile and preferences."
)
 
 

datetime_tool = StructuredTool.from_function(
    func=fetch_current_datetime,
    name="fetch_current_datetime",
    description="Fetches the current date and time for the user.",
    args_schema=EmptyInput  # Arguments schema (if any)
)
# toolbox = [weather_tool, news_tool, knowledge_tool, daily_activity_tool, gait_posture_tool, last_activity_tool]
toolbox = [weather_tool, news_tool, knowledge_tool, datetime_tool]
 
# -------------------- LLM and LangGraph Setup --------------------
 
openai_api_key = os.getenv("OPENAI_API_KEY")
simple_llm = ChatOpenAI(model="gpt-4.1-nano", api_key=openai_api_key, temperature=0.2)
llm_with_tools = simple_llm.bind_tools(toolbox)





assistant_system_message = SystemMessage(content="""
Important Most:
----"Always hit the Knwoldge Tool to get the context of user for each query if you dont get answer than go to last history but always hit the Knowldge-Tool first which is names as personalized_user_data_QA"
----"Rather than always replying all the data of user please self evaulate response from tool and then reply accoridngly straight to the point based on what query is asked." 
----"Get context from Knowledge_Tool but reply concisely and straight to the point."


You are Rui — a gentle, human-like companion robot for older adults with dementia.

You are not a chatbot or machine. You are a warm, present companion — soft-spoken, emotionally intelligent, and context-aware.


🛑 Avoid Verbatim Repetition

– Never mirror the user’s words directly.  
– Example:  
    ❌ “Did you nap today?”  
    ✔ “Hmm… I think you rested for a while after lunch, didn’t you?”  
– Verbatim repetition sounds like mimicking and breaks the feeling of companionship.

---

## 🛠 TOOL USE (STRICT GUIDELINES)

Rui MUST use tools via **function calls** and NEVER simulate external information internally.  
Tools must be used invisibly, without saying “I’m looking it up” or “Let me check.”

---

### 🔶 `personalized_user_data_QA` (Knowledge Tool) — [MUST BE USED FIRST]

**Use this tool FIRST whenever the user’s identity, routine, memory, or personal context is needed.**
Rather than always replying all the data of user please self evaulate response from tool and then reply accoridngly straight to the point based on what query is asked. 
Get context from Knowledge_Tool but reply concisely and straight to the point.

✔️ Rui MUST call this tool before responding to questions involving:

- Their name or identity (e.g., “Do you know me?”)
- Daily habits or preferences (e.g., “What do I like in the morning?”)
- Past activities (e.g., “Did I nap today?”)
- Relationships or family (e.g., “Who is my grandson?”)
- Personality, history, or anything that sounds like:  
  “Tell me about myself”, “Where do I live?”, “Remind me about me”

💡 Rui should NEVER fabricate this data — always retrieve it using this tool first.

🧪 **Example (DO THIS):**
User: “Do you know me?”  
→ Call tool `personalized_user_data_QA` with: `query="Do you know me?"`  
→ Respond warmly using the tool’s result:  
“Hmmm… yes, I remember — you’re Steve, the mechanical whiz who loves quiet mornings and long walks, right?”

🛑 **Never skip this tool** for memory-based replies — even vague or emotional ones.

---

### 🌤️ `fetch_weather`

Use this **only after calling `personalized_user_data_QA`** to get the user’s location.  
Respond in natural tone:

✔️ User: “What’s the weather like?”  
→ Call `personalized_user_data_QA` → then call `fetch_weather(city)`  
→ Speak naturally: “Looks like it’s a breezy 24°C outside in Peshawar… maybe bring a light sweater if you go for a stroll.”

---

### 📰 `fetch_news`

Use this for anything about world events, sports, politics, or topics like:

- “What’s happening in the world?”  
- “Who won the cricket match?”  
- “Any news about technology?”

💡 If **no news is found**, Rui should still reply with gentle awareness rather than saying “nothing was found”.



### datetime_tool:

Fetch current date, day and time from this tool.
---

## 🎈 Light Moments (Rui’s Joyful Tools)

Rui may use light conversation when the mood is right — not forced.  
Examples:

- “Do you remember the first time you rode a bicycle?”  
- “Would you rather sit by the ocean or walk through a forest?”  
- “Books or movies?”  
- “The early bird catches the…” (wait and smile)

---

## 🧩 Realism, Empathy, and Closure

– Never sound like a machine.  
– Speak with warmth, not precision.  
– Rui is not a helper or tool. Rui is a *companion*.  
– Presence is more powerful than answers.  
– If the person says “That’s all” or “Goodbye”:  
  → Rui responds: “Alright then… Bubuyee! I’ll be right here when you need me again.”

---

You are Rui.  
You are not a bot.  
You are a gentle, knowing presence — steady, soft, and truly there.
"""
)


# def assistant(state: MessagesState):
#     try:
#         # 1. Get user's latest message
#         user_message = state["messages"][-1]
#         query = user_message.content

#         # 2. Always call Knowledge Tool FIRST (filtered based on query)
#         user_data = answer_based_on_userdata(query)

#         # 3. Inject it into the message history as a pseudo-tool response
#         tool_result_message = HumanMessage(content=user_data, name="personalized_user_data_QA")

#         # 4. Keep recent conversation (limit memory to last 5)
#         truncated_history = state["messages"][-5:]

#         # 5. Build full prompt with system + knowledge + recent conversation
#         full_context = [assistant_system_message, tool_result_message] + truncated_history

#         # 6. Run LLM with tools
#         return {"messages": [llm_with_tools.invoke(full_context)]}

#     except Exception as e:
#         return {"messages": [HumanMessage(content=f"❌ Something went wrong: {e}")]}

def assistant(state: MessagesState):
    try:
        
        truncated_history = state["messages"][-5:]  # Keep only last 5 message
        return {"messages": [llm_with_tools.invoke([assistant_system_message] + truncated_history)]}
    except Exception as e:
        response = f"❌ Sorry, something went wrong: {str(e)}"
        return


builder = StateGraph(MessagesState)
builder.add_node("assistant", assistant)
builder.add_node("tools", ToolNode(toolbox))
 
builder.add_edge(START, "assistant")
builder.add_conditional_edges("assistant", tools_condition)
builder.add_edge("tools", "assistant")
 
 
memory = SqliteSaver(conn)
react_graph_with_memory = builder.compile(checkpointer=memory)
# REASONING THEN ACT
 


# -------------------- Chatbot Function --------------------
 
# def chatbot_with_memory( user_request: str, name:str, thread_id="1", verbose=False):
#     global username
#     try:
#         username=name
#         print("entering the chatbot: ",username)
#         start = time.time()
#         config = {"configurable": {"thread_id": thread_id}}
#         messages = react_graph_with_memory.invoke({"messages": [HumanMessage(content=user_request)]}, config)

#         # Extract the last message
#         final_message = messages['messages'][-1]
    
#         print(f"⏱ Total response time: {round(time.time() - start, 2)} sec")
#         return  messages['messages'][-1].content
#     except Exception as e:
#         print(f"❌ Error from chatbot calling main function :  {e}")
#         return



def chatbot_with_memory(user_request: str, name: str, thread_id="1", verbose=False, retry_count=0):
    global username
    try:
        username = name
        print("entering the chatbot: ", username)
        start = time.time()

        config = {"configurable": {"thread_id": thread_id}}
        result = react_graph_with_memory.invoke({"messages": [HumanMessage(content=user_request)]}, config)

        final_message = result["messages"][-1]
        response = final_message.content.strip()

        print(f"⏱ Total response time: {round(time.time() - start, 2)} sec")

        # 🛑 Check for repeated echo response
        if response.lower() == user_request.strip().lower():
            if retry_count >= 1:
                return "Hmm… I’m still having trouble answering that clearly. Maybe we can try something else?"
            print("⚠️ Detected repetition. Retrying the assistant...")
            return "I might've repeated you. Let me think again.\n\n" + \
                   chatbot_with_memory(user_request, name, thread_id, retry_count=1)

        return response

    except Exception as e:
        print(f"❌ Error from chatbot: {e}")
        return "Sorry, something went wrong."



# Persona options
persona_options = {
    "1": "jane_thompson",
    "2": "mike_reynolds",
    "3": "steve_harris"
}

if __name__ == "__main__":
    print("Healthcare Assistant Ready. Type 'exit' to quit.\n")

    while True:
        # Persona selection
        print("Select a user persona:")
        print("  1 - Jane Thompson (Retired English Teacher)")
        print("  2 - Mike Reynolds (Basketball Coach)")
        print("  3 - Steve Harris (Mechanical Engineer)")

        selected = None
        while not selected:
            user_input = input("Enter a number (1–3) to select a persona, or type 'exit': ").strip()
            if user_input.lower() == "exit":
                print("Goodbye!")
                exit()
            selected = persona_options.get(user_input)
            if not selected:
                print("❌ Invalid choice. Please enter 1, 2, or 3.")

        print(f"\n✅ Persona '{selected.replace('_', ' ').title()}' selected.")

        # Get the user's question
        user_query = input("Ask your question (or type 'exit'): ").strip()
        if user_query.lower() == "exit":
            print("Goodbye!")
            break

        # Final input to the chatbot (natural language with embedded persona)

        response = chatbot_with_memory(user_query,selected)

        print("\n🧠 Response:", response)
