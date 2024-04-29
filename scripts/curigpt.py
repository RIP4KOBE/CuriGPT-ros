"""
CuriGPT
===========================
High-level task reasoning for CURI manipulation via Multimodal Large Language Models (Qwen-VL-Max).
"""

from http import HTTPStatus
from dashscope import MultiModalConversation
from PIL import Image, ImageDraw
from scripts.audio_assistant import AudioAssistant
import dashscope
import json
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import pyrealsense2 as rs
import cv2
import time
import numpy as np
import os


def save_color_images():
    """
    Capture and save color images from the RealSense camera.

    Parameters:
        folder_path (str): folder to save the images.
        save_interval (int): interval in seconds to save images.
    """

    folder_path = "../assets/img/realtime_rgb"

    # ensure the folder exists
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    # set up the RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # start the pipeline
    pipeline.start(config)
    save_interval = 1
    save_times = 2

    try:
        for i in range(save_times):
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

            if not color_frame:
                print("No color frame captured")
                continue

            color_image = np.asanyarray(color_frame.get_data())
            img_name = "realtime_rgb.png"
            cv2.imwrite(os.path.join(folder_path, img_name), color_image)
            time.sleep(save_interval)

        print(f"RGB image saved")

    finally:
        # stop the pipeline and close the OpenCV windows
        pipeline.stop()
        cv2.destroyAllWindows()

def add_bbox_patch(ax, draw, bbox, color, w, h, caption):
    """
    Add a bounding box patch to the plot.

    Parameters:
        ax (matplotlib.axes.Axes): The axes to add the patch to.
        draw (ImageDraw.Draw): The ImageDraw object to draw on the image.
        bbox (list): The bounding box coordinates.
        color (str): The color of the bounding box.
        w (int): The width of the image.
        h (int): The height of the image.
        caption (str): The caption to annotate the bounding box with.
    """
    # Draw the bounding box on the image
    x1, y1, x2, y2 = bbox
    x1, y1, x2, y2 = (int(x1 / 1000 * w), int(y1 / 1000 * h), int(x2 / 1000 * w), int(y2 / 1000 * h))
    rect = patches.Rectangle((x1, y1), x2 - x1, y2 - y1,
                             linewidth=1, edgecolor=color, facecolor='none')
    ax.add_patch(rect)

    # Annotate the bounding box with a caption
    ax.text(x1, y1, caption, color=color, weight='bold', fontsize=8,
            bbox=dict(facecolor='white', alpha=0.75, edgecolor='none', boxstyle='round,pad=0.1'))

def plot_image_with_bbox(image_path, response):
    """
    Parse the bbox coordinates from the response of CuriGPT and plot them on the image.

    Parameters:
    image_path (str): The path to the image file.
    response (str): The response from the multimodal large language model.
    """

    # parse the JSON string.
    try:
        output_data = json.loads(response)
    except json.JSONDecodeError as e:
        print(f"Error decoding JSON: {e}")
        return

    # load the image.
    image = Image.open(image_path)
    draw = ImageDraw.Draw(image)
    # get the width and height od the image size
    w, h = image.size

    # create a plot.
    fig, ax = plt.subplots()

    # display the image.
    ax.imshow(image)

    # go through each action and plot the bounding boxes.
    if output_data['robot_actions'] is not None:
        for action in output_data['robot_actions']:
            bbox1 = action['parameters']['arg1']['bbox_coordinates']
            caption1 = action['parameters']['arg1']['description']

            # check if arg2 exists
            if 'arg2' in action['parameters']:
                bbox2 = action['parameters']['arg2']['bbox_coordinates']
                caption2 = action['parameters']['arg2']['description']

                add_bbox_patch(ax, draw, bbox1, 'red', w, h, caption1)
                add_bbox_patch(ax, draw, bbox2, 'blue', w, h, caption2)

            else:
                add_bbox_patch(ax, draw, bbox1, 'red', w, h, caption1)

        # show the plot with the bounding boxes.
        plt.show()


def single_multimodal_call(base_prompt, query, prompt_img_path, log=True, return_response=True):
    """
    Single round multimodal conversation call with CuriGPT.

    Parameters:
        base_prompt (list): The base prompt for multimodal reasoning.
        query (str): The user query.
        realtime_img_path (str): The path to the real-time image.
        log (bool): Whether to log the response.
        return_response (bool): Whether to return the response.
    """
    # make a copy of the base prompt to create a new prompt
    new_prompt = base_prompt.copy()
    prompt_img_path = prompt_img_path
    new_prompt.append({"role": "user", "content": [{"image": prompt_img_path}, {"text": query}]})

    # call with qwen-vl-max model
    # response_dict = dashscope.MultiModalConversation.call(model='qwen-vl-max',
    #                                                  messages=new_prompt)

    # call with qwen-vl-chat-v1 model
    response_dict = MultiModalConversation.call(model=MultiModalConversation.Models.qwen_vl_chat_v1,
                                               messages=new_prompt, top_p=0.9, top_k=100)

    # call with qwen-vl-plus model
    # response_dict = MultiModalConversation.call(model='qwen-vl-plus',
    #                                            messages=new_prompt, top_p=0.1, top_k=10)

    if response_dict.status_code == HTTPStatus.OK:
        response = response_dict[
            "output"]["choices"][0]["message"]["content"]
        print("CURI response:\n", response)

        # plot the image with bounding boxes if the response contains actions.
        # plot_image_with_bbox(prompt_img_path, response)

    else:
        print(response_dict.code)  # The error code.
        print(response_dict.message)  # The error message.

    if return_response:
        return response


def multiple_multimodal_call(base_prompt, query, prompt_img_path, rounds=10, log=True, return_response=False):
    """
    Multiple rounds of  multimodal conversation call with CuriGPT.

    Parameters:
        base_prompt (list): The base prompt for multimodal reasoning.
        query (str): The user query.
        rounds (int): The number of multiple rounds.
        log (bool): Whether to log the response.
        return_response (bool): Whether to return the response.
    """

    new_prompt = base_prompt.copy()
    prompt_img_path = prompt_img_path

    for i in range(rounds):
        new_prompt.append({"role": "user", "content": [{"image": prompt_img_path}, {"text": query}]})

        # call with qwen-vl-max model
        # response_dict = dashscope.MultiModalConversation.call(model='qwen-vl-max',
        #                                                  messages=new_prompt)

        # call with qwen-vl-chat-v1 model
        response_dict = MultiModalConversation.call(model=MultiModalConversation.Models.qwen_vl_chat_v1,
                                                    messages=new_prompt, top_p=0.9, top_k=100)

        # call with qwen-vl-plus model
        # response_dict = MultiModalConversation.call(model='qwen-vl-plus',
        #                                            messages=new_prompt, top_p=0.1, top_k=10)

        response = response_dict[
            "output"]['choices'][0]['message']['content']

        if response_dict.status_code == HTTPStatus.OK:
            print("CURI response:\n", response)
        else:
            print(response_dict.code)  # The error code.
            print(response_dict.message)  # The error message.

        if return_response:
            return response

def get_curi_response(base_multimodal_prompt, rounds=10, prompt_append=False):

    """Get CURI response for a given number of rounds."""
    inference_results = None
    if not prompt_append:
        print("without prompt append")
        for i in range(rounds):
            instruction = input("Please input your instruction: ")
            inference_results = single_multimodal_call(base_multimodal_prompt, instruction, log=True, return_response=False)

    else:
        if rounds < 1:
            raise ValueError("Number of rounds must be at least 1.")

        for i in range(rounds):
            instruction = input("Please input your instruction: ")

            # Single call for each round
            if i == 1:
                # For the first round or if not returning response, just update the prompt
                inference_results = single_multimodal_call(base_multimodal_prompt, instruction, log=True,
                                                           return_response=False)

            else:
                # For subsequent rounds, get the response as well to append to the prompt
                new_prompt, response_dict = single_multimodal_call(base_multimodal_prompt, instruction, log=True,
                                                                   return_response=True)
                if 'output' in response_dict and 'choices' in response_dict['output'] and len(
                        response_dict['output']['choices']) > 0:
                    # Append the model's response to the prompt for the next round
                    response_content = response_dict['output']['choices'][0]['message']['content']
                    response_role = response_dict['output']['choices'][0]['message']['role']
                    new_prompt.append({'role': response_role, 'content': [{'text': response_content}]})
                base_multimodal_prompt = new_prompt

def get_curi_response_with_audio(api_key, base_url, user_input, curigpt_output, realtime_img_path, local_img_path,
                                 base_multimodal_prompt, rounds=10, realtime_flag=True,prompt_append=False):
    """
    Get CURI response with audio input and output.

    Parameters:
        api_key (str): The OpenAI API key.
        base_url (str): The base URL for the OpenAI API.
        user_input (str): The path to the user audio input file.
        curigpt_output (str): The path to the CuriGPT audio output file.
        realtime_img_path (str): The path to the real-time image.
        local_img_path (str): The path to the local image.
        base_multimodal_prompt (list): The base prompt for multimodal reasoning.
        rounds (int): The number of rounds.
        realtime_flag (bool): Whether to enable interactive reasoning in real-time.
        prompt_append (bool): Whether to append the model's current response to the next prompt.
    """
    # Create an instance of the AudioAssistant class
    assistant = AudioAssistant(api_key, base_url, user_input, curigpt_output)

    # check if the CuriGPT need to work in the real-time mode
    if realtime_flag:
        prompt_img_path = realtime_img_path
    else:
        prompt_img_path = local_img_path

    inference_results = None
    if not prompt_append:
        # print("without prompt append")
        for i in range(rounds):
            save_color_images()
            assistant.record_audio()
            transcription = assistant.transcribe_audio()
            instruction = transcription
            response = single_multimodal_call(base_multimodal_prompt, instruction, prompt_img_path, log=True, return_response=True)

            print("CURI audio response:\n", response)

            # play the audio response
            if response is not None:
                assistant.text_to_speech(response)

    else:
        if rounds < 1:
            raise ValueError("Number of rounds must be at least 1.")

        for i in range(rounds):
            instruction = input("Please input your instruction: ")

            # Single call for each round
            if i == 1:
                # for the first round or if not returning response, just update the prompt
                inference_results = single_multimodal_call(base_multimodal_prompt, instruction, log=True,
                                                           return_response=False)

            else:
                # For subsequent rounds, get the response as well to append to the prompt
                new_prompt, response_dict = single_multimodal_call(base_multimodal_prompt, instruction, log=True,
                                                                   return_response=True)
                if 'output' in response_dict and 'choices' in response_dict['output'] and len(
                        response_dict['output']['choices']) > 0:
                    # Append the model's response to the prompt for the next round
                    response_content = response_dict['output']['choices'][0]['message']['content']
                    response_role = response_dict['output']['choices'][0]['message']['role']
                    new_prompt.append({'role': response_role, 'content': [{'text': response_content}]})
                base_multimodal_prompt = new_prompt


if __name__ == '__main__':

    # design the base prompt for multimodal reasoning
    base_multimodal_prompt = [
        # {
        #     "role": "user",
        #     "content": [
        #         {"image": local_file_path1},
        #         {"text": "下午好, 你能描述下你现在看到的场景吗?"},
        #     ]
        # },
        # {
        #     "role": "assistant",
        #     "content": [{
        #         "text": "好的，我看到一个白色的桌子上放着一些物品。左上角有一个橙色的洗涤剂，中间有一个绿色的绿茶罐子，一个蓝色的午餐肉罐头和一个红色的盘子，盘子里放着一根香蕉。"}]
        # },
        # {
        #     "role": "user",
        #     "content": [
        #         {"image": local_file_path1},
        #         {"text": "你觉得桌上的哪个物品适合作为吃火锅时的配菜呢？"},
        #     ]
        # },
        # {
        #     "role": "assistant",
        #     "content": [{"text": "我认为午餐肉罐头适合作为吃火锅时的配菜，因为它易于烹饪，可以在火锅中快速煮熟，并可以与其他食材搭配，增加火锅的多样性"}]
        # },
        # # {
        # #     "role": "user",
        # #     "content": [
        # #         {"image": local_file_path1},
        # #         {"text": "那香蕉为什么不适合呢？"},
        # #     ]
        # # },
        # # {
        # #     "role": "assistant",
        # #     "content": [{"text": "因为香蕉是一种水果，其口感和味道与火锅的其他食材相比可能会显得不太协调。此外，香蕉在高温下容易变软和糊化，这可能会影响其口感和营养价值。因此，香蕉通常不是火锅的常见配菜"}]
        # # },
        # {
        #     "role": "user",
        #     "content": [
        #         {"image": local_file_path1},
        #         {"text": "我想清洗一下红色盘子，你可以把洗涤剂递给我吗"},
        #     ]
        # },
        # {
        #     "role": "assistant",
        #     "content": [{"text": "好的，没问题。我将把洗涤剂瓶子递给你。"}]
        # },
    ]

    # Load the configuration from the config.json file
    with open('../config/config.json', 'r') as config_file:
        config = json.load(config_file)

    # accessing configuration variables
    api_key = config['openai_api_key']
    base_url = config['base_url']
    user_input_filename = config['user_input_filename']
    curigpt_output_filename = config['curigpt_output_filename']
    realtime_img_path = config['realtime_img_path']
    local_img_path = config['local_img_path']

    # get the CURI response with audio input and output
    get_curi_response_with_audio(api_key, base_url, user_input_filename, curigpt_output_filename, realtime_img_path,
                                 local_img_path, base_multimodal_prompt, rounds=10, realtime_flag=True, prompt_append=False)